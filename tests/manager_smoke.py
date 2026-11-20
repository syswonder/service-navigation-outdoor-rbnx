"""Docker-only, no original repository needed: real worker lifecycle + virtual sensors."""

import json
import math
import os
from pathlib import Path
import threading
import time

if not Path("/.dockerenv").exists():
    raise SystemExit("Docker only")
os.environ.update(
    ROS_DOMAIN_ID="95", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
)
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
from outdoor_nav.manager import RuntimeManager

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / "rbnx-build/manager-smoke" / time.strftime("%Y%m%d-%H%M%S")
RUN.mkdir(parents=True)
OUTPUT = "/service_test/velocity"


class VirtualSensors(Node):
    def __init__(self):
        super().__init__("service_virtual_inputs")
        self.x = 0.0
        self.v = 0.0
        self.at = 0.0
        self.nonzero = 0
        self.odom = self.create_publisher(Odometry, "/service_test/odom", 10)
        self.geo = self.create_publisher(String, "/service_test/localization", 10)
        self.tf = self.create_publisher(TFMessage, "/service_test/tf", 10)
        self.cloud = self.create_publisher(PointCloud2, "/service_test/cloud", 10)
        self.create_subscription(Twist, OUTPUT, self.command, 10)
        self.fields = [
            PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(("x", "y", "z"))
        ]
        self.rays = [
            (7.5 * math.cos(i * math.pi / 360), 7.5 * math.sin(i * math.pi / 360), 0.0)
            for i in range(720)
        ]
        self.create_timer(0.05, self.tick)

    def command(self, msg):
        self.v = msg.linear.x
        self.at = time.monotonic()
        if abs(msg.linear.x) + abs(msg.angular.z) > 1e-6:
            self.nonzero += 1

    def tick(self):
        if time.monotonic() - self.at > 0.4:
            self.v = 0.0
        self.x += self.v * 0.05
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = self.v
        self.odom.publish(odom)
        transforms = []
        for parent, child, x in (("map", "odom", 0.0), ("odom", "base_link", self.x)):
            tr = TransformStamped()
            tr.header.stamp = stamp
            tr.header.frame_id = parent
            tr.child_frame_id = child
            tr.transform.translation.x = x
            tr.transform.rotation.w = 1.0
            transforms.append(tr)
        self.tf.publish(TFMessage(transforms=transforms))
        self.geo.publish(
            String(
                data=json.dumps(
                    {
                        "ready": True,
                        "grade": "fixed",
                        "extrinsics": {"ready": True},
                        "ownership_conflict": False,
                    }
                )
            )
        )
        self.cloud.publish(
            point_cloud2.create_cloud(
                Header(stamp=stamp, frame_id="base_link"), self.fields, self.rays
            )
        )
        assert (
            self.count_publishers("/cmd_vel") == 0
            and self.count_publishers("/NAV_CMD") == 0
        )


rclpy.init()
sensors = VirtualSensors()
ex = SingleThreadedExecutor()
ex.add_node(sensors)
thread = threading.Thread(target=ex.spin, daemon=True)
thread.start()
manager = RuntimeManager()
report = {
    "robot_connection": False,
    "baseline_runtime_imported": False,
    "passed": False,
}
try:
    config = {
        "params_file": str(ROOT / "config/nav2_params.example.yml"),
        "run_dir": str(RUN),
        "http_port": 19976,
        "velocity_output_topic": OUTPUT,
        "global_grid": {
            "origin": [-10.0, -10.0],
            "width": 40.0,
            "height": 40.0,
            "resolution": 0.1,
        },
        "topic_remap": {
            "odom": "/service_test/odom",
            "cloud": "/service_test/cloud",
            "scan": "/service_test/scan",
            "localization": "/service_test/localization",
            "tf": "/service_test/tf",
            "tf_static": "/service_test/tf_static",
        },
        "policy": {
            "motion_enabled": True,
            "calibrated": True,
            "antenna_offset_confirmed": True,
            "footprint_confirmed": True,
            "cloud_frame_confirmed": True,
            "odometry_confirmed": True,
            "localization_mode": "external_odom",
            "heading_source": "configured",
            "require_chassis_ready": False,
        },
    }
    manager.start(config)
    assert not manager.request("/api/status")["health"]["service_active"]
    report["init_inactive"] = True
    manager.active(True)
    deadline = time.monotonic() + 10
    while not manager.request("/api/status")["ready"]:
        if time.monotonic() > deadline:
            raise RuntimeError(manager.request("/api/status"))
        time.sleep(0.1)
    payload = {
        "frame_id": "map",
        "reference_path": [{"x": 0.0, "y": 0.0}, {"x": 7.0, "y": 0.0}],
        "goal": {"x": 7.0, "y": 0.0},
        "request_id": "a" * 32,
    }
    manager.request("/api/tasks", payload)
    deadline = time.monotonic() + 70
    while True:
        task = manager.request("/api/tasks/" + "a" * 32)
        if task["state"] in ("succeeded", "failed", "canceled"):
            break
        if time.monotonic() > deadline:
            raise TimeoutError(task)
        time.sleep(0.1)
    assert task["state"] == "succeeded" and sensors.nonzero > 0, task
    report["arrival"] = task["state"]
    report["nonzero_virtual_commands"] = sensors.nonzero
    x = manager.request("/api/status")["pose"]["x"]
    manager.request(
        "/api/tasks",
        {
            "frame_id": "map",
            "reference_path": [{"x": x, "y": 0.0}, {"x": x + 12.0, "y": 0.0}],
            "goal": {"x": x + 12.0, "y": 0.0},
            "request_id": "b" * 32,
        },
    )
    deadline = time.monotonic() + 5
    while manager.request("/api/tasks/" + "b" * 32)["state"] != "navigating":
        if time.monotonic() > deadline:
            raise TimeoutError("No second execution")
        time.sleep(0.1)
    manager.active(False)
    assert manager.request("/api/tasks/" + "b" * 32)["state"] == "canceled"
    assert not manager.request("/api/status")["health"]["service_active"]
    assert abs(sensors.v) <= 0.03
    report["deactivate_stopped"] = True
    manager.shutdown()
    report["shutdown_reaped"] = manager.process is None
    report["passed"] = True
finally:
    manager.shutdown()
    ex.shutdown()
    thread.join(timeout=3)
    sensors.destroy_node()
    rclpy.shutdown()
    (RUN / "report.json").write_text(json.dumps(report, indent=2))
    print("MANAGER_SMOKE_REPORT", json.dumps(report), str(RUN), flush=True)
