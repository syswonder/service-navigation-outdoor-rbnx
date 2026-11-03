"""ROS execution worker. Sensors/localization are external providers."""

import argparse
import json
import math
from pathlib import Path
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from action_msgs.srv import CancelGoal
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .configuration import load_runtime
from .core.scan_health import healthy_scan
from .core.tasks import TaskStore, TERMINAL
from .executor import Executor, LATCHED
from .http import serve, ApiError
from .requests import submit


class OutdoorExecutor(Executor):
    def __init__(self, runtime_file):
        # Do not start the upstream business API, scene loader or localizer.
        from rclpy.node import Node

        Node.__init__(self, "outdoor_execution")
        self.runtime = load_runtime(runtime_file)
        self.config = self.runtime["policy"]
        self.hardware = True
        self.base_frame = "base_link"
        self.arrival_tolerance = 0.5
        self.diagnostic = False  # Output isolation is an explicit topic remap, not a readiness failure.
        self.group = ReentrantCallbackGroup()
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.tasks = TaskStore(journal=Path(self.runtime["run_dir"]) / "tasks.json")
        self.stats, self.lifecycle, self.lifecycle_pending = {}, {}, set()
        self.clock_value, self.clock_changed = None, 0.0
        self.clock_rewound = False
        self.global_map = self.local_map = None
        self.costmap_view_cache = {}
        self.raw_command, self.raw_received = Twist(), 0.0
        self.velocity, self.velocity_at = (math.inf, math.inf), 0.0
        self.owner, self.stopping = None, True
        self.stop_failure = None
        self.handles, self.requests, self.cancel_sent = {}, {}, set()
        self.state_lock = threading.RLock()
        self.geo, self.geo_at = {}, -math.inf
        self.guard, self.guard_at = {}, -math.inf
        self.chassis, self.chassis_at = {}, -math.inf
        self.service_active = False
        self.startup_stopped, self.stationary_since = False, None
        self.path_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose", callback_group=self.group
        )
        self.follow_client = ActionClient(
            self, FollowPath, "follow_path", callback_group=self.group
        )
        self.navigate_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.group
        )
        self.cancel_client = self.create_client(
            CancelGoal,
            "navigate_to_pose/_action/cancel_goal",
            callback_group=self.group,
        )
        self.lifecycle_clients = {
            n: self.create_client(
                GetState, f"/{n}/get_state", callback_group=self.group
            )
            for n in ("planner_server", "controller_server")
        }
        self.command_pub = self.create_publisher(Twist, "/outdoor/safe_command", 10)
        self.path_pub = self.create_publisher(
            NavPath, "/navigation/execution_path", LATCHED
        )
        for kind, name, callback, qos in (
            (OccupancyGrid, "/global_costmap/costmap", self.on_global, LATCHED),
            (OccupancyGrid, "/local_costmap/costmap", self.on_local, LATCHED),
            (Odometry, "/outdoor/odom", self.on_odom, 10),
            (Twist, "/navigation/controller_command", self.on_command, 10),
            (String, "/outdoor/localization", self.on_geo, 10),
            (String, "/outdoor/command_status", self.on_guard, 10),
            (String, "/outdoor/chassis_status", self.on_chassis, 10),
            (LaserScan, "/outdoor/scan", self.on_scan, qos_profile_sensor_data),
        ):
            self.create_subscription(kind, name, callback, qos)
        self.server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            handle_accepted_callback=self.on_accepted,
            callback_group=self.group,
        )
        for interval, callback in (
            (0.05, self.safety_tick),
            (0.1, self.dispatch),
            (1.0, self.poll_lifecycle),
            (0.1, self.check_stationary),
        ):
            self.create_timer(
                interval, callback, clock=self.steady, callback_group=self.group
            )
        self.controller_parameters = self.create_client(
            SetParametersAtomically,
            "/controller_server/set_parameters_atomically",
            callback_group=self.group,
        )
        self.obstacles = self.places = None
        self.http = serve(
            self, port=self.runtime.get("http_port", 19975), host="127.0.0.1"
        )

    def check_stationary(self):
        now = time.monotonic()
        stopped = (
            now - self.velocity_at < self.config["sensor_timeout_s"]
            and self.velocity[0] <= self.config["stopped_linear_mps"]
            and self.velocity[1] <= self.config["stopped_angular_rps"]
        )
        self.stationary_since = (
            (now if self.stationary_since is None else self.stationary_since)
            if stopped
            else None
        )
        if (
            self.stationary_since is not None
            and now - self.stationary_since >= self.config["stop_hold_s"]
        ):
            self.startup_stopped = True

    def on_geo(self, msg):
        self._state(msg, "geo")

    def on_guard(self, msg):
        self._state(msg, "guard")

    def on_chassis(self, msg):
        self._state(msg, "chassis")

    def _state(self, msg, name):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError("Expected object")
            setattr(self, name, value)
            setattr(self, name + "_at", time.monotonic())
        except (ValueError, TypeError):
            setattr(self, name, {})

    def on_scan(self, msg):
        if healthy_scan(
            msg,
            self.get_clock().now().nanoseconds / 1e9,
            self.config["sensor_timeout_s"],
        ):
            self.mark("lidar")

    def status(self, include_task=True):
        now = time.monotonic()
        c = self.config
        g = self.geo
        t = c["sensor_timeout_s"]
        owned = (
            self.tasks.snapshot(self.owner, include_paths=False) if self.owner else None
        )
        required = owned.get("localization_grade") if owned else None
        health = {
            k: now - self.stats.get(k, -math.inf) < limit
            for k, limit in (("lidar", t), ("global_map", 3), ("local_map", t))
        }
        health.update(
            localization=bool(g.get("ready"))
            and now - self.geo_at < t
            and (
                required != "fixed"
                or g.get("grade") == "fixed"
                or (g.get("grade") == "float" and c["allow_float_during_fixed_task"])
            ),
            heading=bool(g.get("heading", {}).get("ready"))
            if c["heading_source"] == "wit_imu"
            else True,
            calibration=all(
                c[k]
                for k in (
                    "calibrated",
                    "antenna_offset_confirmed",
                    "footprint_confirmed",
                    "cloud_frame_confirmed",
                )
            ),
            motion_enabled=c["motion_enabled"],
            chassis=not c["require_chassis_ready"]
            or (self.chassis.get("ready") is True and now - self.chassis_at < t),
            production_mode=True,
            sensor_extrinsics=bool(g.get("extrinsics", {}).get("ready")),
            service_active=self.service_active,
            controller_stopped=self.startup_stopped and self.stop_failure is None,
            odometry=(
                c["feedback_confirmed"]
                if c["localization_mode"] == "rtk_imu"
                else c["odometry_confirmed"]
            )
            and now - self.velocity_at < t,
            actions=self.path_client.server_is_ready()
            and self.follow_client.server_is_ready(),
            exclusive_owners=not g.get("ownership_conflict", False)
            and not self.guard.get("conflict", False),
            command_guard=now - self.guard_at < t
            and self.guard.get("output_topic") == self.runtime["velocity_output_topic"],
        )
        for name in self.lifecycle_clients:
            value, stamp = self.lifecycle.get(name, (False, 0.0))
            health[name] = value and now - stamp < 5
        pose = self.pose()
        health["tf"] = pose is not None and pose["age_s"] < t
        return {
            "schema_version": 1,
            "ready": all(health.values()),
            "health": health,
            "localization_grade": g.get("grade"),
            "heading": g.get("heading"),
            "feedback": g.get("feedback"),
            "pose": pose,
            "command_topic": self.runtime["velocity_output_topic"],
            "not_ready_reasons": [k for k, v in health.items() if not v],
            "measured_velocity": list(self.velocity)
            if all(math.isfinite(v) for v in self.velocity)
            else None,
            "velocity_fresh": now - self.velocity_at < t,
            "task": self.tasks.snapshot() if include_task else None,
        }

    def submit(self, payload):
        return submit(self, payload)

    def set_active(self, active):
        with self.state_lock:
            self.service_active = active
            if not active:
                self.stopping = True
                for task in list(self.tasks.tasks.values()):
                    if task.state not in TERMINAL:
                        self.tasks.cancel(task.task_id)
                self.command_pub.publish(Twist())

    def on_goal(self, request):
        p = request.pose.pose.position
        s = self.tasks.snapshot()
        return (
            GoalResponse.ACCEPT
            if s
            and s["state"] == "accepted"
            and s["goal"]["x"] == p.x
            and s["goal"]["y"] == p.y
            and self.status(False)["ready"]
            else GoalResponse.REJECT
        )

    def on_accepted(self, handle):
        if bytes(handle.goal_id.uuid).hex() not in self.tasks.tasks:
            handle.execute()
            return
        return super().on_accepted(handle)

    async def execute(self, handle):
        if bytes(handle.goal_id.uuid).hex() not in self.tasks.tasks:
            handle.abort()
            return NavigateToPose.Result()
        return await super().execute(handle)

    def arrival_heading_ok(self, position, goal):
        return (
            "yaw" not in goal
            or abs(
                math.atan2(
                    math.sin(position["yaw"] - goal["yaw"]),
                    math.cos(position["yaw"] - goal["yaw"]),
                )
            )
            <= 0.20
        )

    async def prepare_route(self, route, task_id):
        if not await self.confirm_stop():
            raise RuntimeError("Cannot confirm stationary robot before route execution")
        tolerance = self.tasks.snapshot(task_id)["arrival_tolerance_m"]
        if tolerance is None:
            raise RuntimeError("Task has no GNSS acceptance policy")
        if not self.controller_parameters.service_is_ready():
            raise RuntimeError("Controller parameter service is unavailable")
        req = SetParametersAtomically.Request()
        req.parameters = [
            Parameter(
                "goal_checker.xy_goal_tolerance", value=float(min(tolerance, 0.5))
            ).to_parameter_msg()
        ]
        result = await self.result(self.controller_parameters.call_async(req), task_id)
        if not result.result.successful:
            raise RuntimeError("Controller rejected the task arrival tolerance")

    async def guided_path(self, guidance, window, position, task_id):
        path = await super().guided_path(guidance, window, position, task_id)
        goal = self.tasks.snapshot(task_id)["goal"]
        if (
            path
            and path.poses
            and window[-1] == len(guidance.points) - 1
            and "yaw" in goal
        ):
            q = path.poses[-1].pose.orientation
            q.z, q.w = math.sin(goal["yaw"] / 2), math.cos(goal["yaw"] / 2)
        return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime", required=True)
    args, ros = parser.parse_known_args()
    rclpy.init(args=ros, signal_handler_options=SignalHandlerOptions.NO)
    node = OutdoorExecutor(args.runtime)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    stopping = threading.Event()

    def stop(*_):
        node.set_active(False)
        stopping.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while not stopping.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown(timeout_sec=20)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
