"""Run the deployed Ranger chain on offline-descent virtual sensors and plant."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from uuid import uuid4

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from std_msgs.msg import Header, String

from outdoor_nav.configuration import TOPICS
from outdoor_nav.manager import RuntimeManager
from semantic_local_grid.ros_utils import cloud_message

OUTPUT = "/ranger_smoke/final_command"
SCAN = "/ranger_smoke/scan"
READY = String(data='{"ready":true,"grade":"fixed","extrinsics":{"ready":true}}')


def wait(check, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = check()
        if result:
            return result
        time.sleep(.08)
    raise TimeoutError("Ranger workflow did not reach its expected state")


def trial(args):
    sys.path.append(str(args.offline))
    from live_descent import TRIAL_SCENES, stair_margin, tick
    from live_descent_planner import Pose
    from live_descent_world import GOAL, START
    from ranger_ros_depth import OPTICAL, RegisteredRanger, SensorFixture, rigid_message

    scene = TRIAL_SCENES["flat_control" if args.case in {"depth_only_outage", "disabled"} else args.case]
    world = RegisteredRanger(scene, 77291, 30.)
    enabled = args.case != "disabled"
    goal = (-2.6, 0.) if not enabled else GOAL
    bearings = np.linspace(-math.pi, math.pi, 181)
    background = np.column_stack((7.8*np.cos(bearings), 7.8*np.sin(bearings),
                                  np.full(len(bearings), .60-world.base_height)))

    class Plant(SensorFixture):
        def __init__(self):
            super().__init__(world)
            self.pose = Pose(*START, float(world.ground(*START)), 0.)
            self.now = time.monotonic()
            self.command = Twist()
            self.turned = 0.
            self.error = None
            self.on_ramp = False
            self.outage_at = None
            self.outage_xy = None
            self.zero_at = None
            self.spacing = math.inf
            self.projection_status = None
            self.supported_cells = 0
            self.got_motion = False
            self.odom = self.create_publisher(Odometry, TOPICS["odom"], 10)
            self.geo = self.create_publisher(String, TOPICS["localization"], 10)
            self.lidar = self.create_publisher(PointCloud2, TOPICS["cloud"], qos_profile_sensor_data)
            self.scan = self.create_publisher(LaserScan, SCAN, qos_profile_sensor_data)
            self.publish_static()
            self.create_timer(1., self.publish_static)
            self.create_subscription(String, "/outdoor/ranger/projection_status", self.on_projection, 10)
            self.create_subscription(OccupancyGrid, "/outdoor/ranger/ground_support", self.on_support, 1)
            self.create_subscription(Twist, OUTPUT, self.on_command, 10)
            self.create_timer(.075, self.advance)

        def publish_static(self):
            stamp = self.get_clock().now().to_msg()
            self.static.sendTransform([
                rigid_message(stamp, "map", "odom", (0., 0., 0.), np.eye(3)),
                rigid_message(stamp, "base_link", "fixture_optical",
                              world.info["mounts_urdf"]["camera_gemini330_link"]["xyz"], world.tilt @ OPTICAL),
            ])

        def on_projection(self, msg):
            self.projection_status = msg.data

        def on_support(self, msg):
            self.supported_cells = max(self.supported_cells, msg.data.count(0))

        def on_command(self, msg):
            self.command = msg
            if self.outage_at and not msg.linear.x and self.zero_at is None:
                self.zero_at = time.monotonic()

        def advance(self):
            now = time.monotonic()
            dt, self.now = min(.2, now-self.now), now
            self.pose, issue = tick(world, self.pose, self.command.linear.x, self.command.angular.z, dt, world.motion)
            self.error = self.error or issue
            self.got_motion |= self.pose.speed > .08
            if scene.ramp:
                self.on_ramp |= (scene.ramp.start + .8 < self.pose.x < scene.ramp.end
                                 and scene.ramp.left < self.pose.y < scene.ramp.right)
            if not scene.flat:
                self.spacing = min(self.spacing, stair_margin(self.pose, world.radius))
            if args.case == "depth_only_outage" and self.got_motion and self.outage_at is None:
                self.outage_at = now
                self.outage_xy = self.pose.xy
            depth, lidar, position, rotation = world.sensor_frame(self.pose)
            stamp = self.get_clock().now().to_msg()
            self.tf.sendTransform(rigid_message(stamp, "odom", "base_link", position, rotation))
            odom = Odometry(header=Header(stamp=stamp, frame_id="odom"), child_frame_id="base_link")
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.pose.speed, self.command.angular.z
            odom.pose.pose.orientation.w = 1.
            self.odom.publish(odom)
            self.geo.publish(READY)
            # Distant background returns keep the original height-filtered scan
            # live on empty terrain. They lie outside the support grid's range.
            raw = np.vstack(((lidar-position) @ rotation, background))
            header = Header(stamp=stamp, frame_id="base_link")
            self.lidar.publish(cloud_message(raw, header))
            # Provide the existing 2-D lidar interface from the same synthetic
            # returns, independently of whether the host's projector republishes.
            ranges = np.full(360, np.inf, dtype=np.float32)
            distance = np.linalg.norm(raw[:, :2], axis=1)
            keep = (.5 <= distance) & (distance < 8.) & (raw[:, 2] + world.base_height >= .2)
            angle = np.arctan2(raw[keep, 1], raw[keep, 0])
            bins = np.floor((angle + math.pi)*180/math.pi).astype(int) % 360
            np.minimum.at(ranges, bins, distance[keep].astype(np.float32))
            self.scan.publish(LaserScan(header=header, angle_min=-math.pi, angle_max=math.pi,
                                        angle_increment=math.pi/180, range_min=.5, range_max=8.,
                                        ranges=ranges.tolist()))
            if enabled and self.outage_at is None:
                c = world.camera
                info = CameraInfo(width=c["width"], height=c["height"], distortion_model="plumb_bob")
                info.header = Header(stamp=stamp, frame_id="fixture_optical")
                info.k = [c["fx"], 0., c["cx"], 0., c["fy"], c["cy"], 0., 0., 1.]
                info.r = np.eye(3).ravel().tolist()
                image = Image(header=info.header, width=c["width"], height=c["height"], encoding="16UC1",
                              step=2*c["width"], data=depth.tobytes())
                self.send_camera(image, info, info)

    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=True)
    ref = args.output / "ground.json"
    ref.write_text(json.dumps({"ground_z_m": -world.base_height}))
    manager = RuntimeManager()
    rclpy.init()
    node = Plant()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        confirmations = ("calibrated", "antenna_offset_confirmed", "footprint_confirmed",
                         "cloud_frame_confirmed", "odometry_confirmed", "camera_frame_confirmed")
        config = {
            "params_file": str(root / "config/nav2_params.example.yml"), "ground_height_file": str(ref),
            "run_dir": str(args.output), "http_port": args.port, "velocity_output_topic": OUTPUT,
            "worker_python": sys.executable,
            "global_grid": {"origin": [-16., -16.], "width": 32., "height": 32., "resolution": .1, "rolling": True},
            "topic_remap": {"tf": "/tf", "scan": SCAN},
            "policy": {
                **dict.fromkeys(confirmations, True),
                "ranger_descent_enabled": enabled, "ranger_descent_mode": "staged",
                "motion_enabled": True, "require_chassis_ready": False,
                "localization_mode": "external_odom", "heading_source": "configured",
                "fixed_arrival_m": .25 if enabled else .40,
                "float_arrival_m": .25 if enabled else .40, "footprint_radius_m": world.radius,
                "stopped_linear_mps": .025, "stop_hold_s": .4, "ranger_depth_timeout_s": .65,
            },
        }
        manager.start(config)
        manager.active(True)
        wait(lambda: manager.request("/api/status")["ready"], 32.)
        target = dict(zip(("x", "y"), goal))
        manager.request("/api/tasks", {"frame_id": "map",
                        "reference_path": [{"x": START[0], "y": START[1]}, target],
                        "goal": target, "request_id": uuid4().hex})

        def finished():
            if node.error:
                raise RuntimeError(node.error)
            status = manager.request("/api/status")
            task = status["task"]
            return task if task and task["state"] in {"failed", "succeeded", "canceled"} else None

        task = wait(finished, args.timeout)
        assert node.pose.speed <= .025, task
        assert not enabled or node.supported_cells > 0
        outage = None
        if args.case == "depth_only_outage":
            assert task["state"] == "failed" and "depth" in task["reason"].lower(), task
            assert node.outage_at and node.zero_at and node.zero_at-node.outage_at < .9, task
            outage = {"zero_delay_s": node.zero_at-node.outage_at,
                      "coast_m": math.dist(node.outage_xy, node.pose.xy)}
            assert outage["coast_m"] <= .25, outage
        elif scene.expected == "no_route":
            assert task["state"] == "failed" and "no confirmed safe route" in task["reason"], task
            assert node.pose.z > -.08, task
        else:
            assert task["state"] == "succeeded", task
            assert math.dist(node.pose.xy, goal) <= (.30 if enabled else .45), task
            assert scene.flat or node.on_ramp, task
        print(json.dumps({"case": args.case, "state": task["state"], "reason": task["reason"],
                          "pose": [node.pose.x, node.pose.y, node.pose.z], "speed": node.pose.speed,
                          "ramp_used": node.on_ramp, "stair_clearance_m": node.spacing if not scene.flat else None,
                          "outage": outage}))
    except Exception:
        if manager.process is not None and manager.process.poll() is None:
            print(manager.request("/api/status"), flush=True)
        print("simulated pose:", node.pose, "motion:", node.got_motion,
              "lidar subscribers:", node.lidar.get_subscription_count(),
              "scan subscribers:", node.scan.get_subscription_count(),
              "camera subscribers:", node.image.get_subscription_count(),
              "confirmed cells:", node.supported_cells, "projection error:", node.projection_status, flush=True)
        raise
    finally:
        manager.shutdown()
        executor.shutdown()
        spinner.join(timeout=3.)
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--offline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port", type=int, default=19987)
    parser.add_argument("--timeout", type=float, default=245.)
    parser.add_argument("case", choices=("left_ramp", "right_offset", "stairs_only", "blocked_ramp",
                                         "depth_only_outage", "disabled"))
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") not in {
        str(domain) for domain in range(210, 220)
    }:
        raise RuntimeError("Run on an isolated local ROS domain 210–219")
    os.environ["no_proxy"] = "localhost,127.0.0.1"
    trial(args)


if __name__ == "__main__":
    main()
