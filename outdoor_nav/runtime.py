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
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt32
from tf2_ros import Buffer, TransformListener

from .configuration import load_runtime
from .core.scan_health import healthy_scan
from .core.gait import GaitSchedule, STAIRS
from .core.scan_gate import stairs_obstacle_off
from std_srvs.srv import Empty
from .core.tasks import TaskStore, TERMINAL
from .executor import Executor, LATCHED
from .http import serve, ApiError
from .requests import submit
from .localization_policy import continuation_allowed, arrival_confirmed


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
        # Pose readiness needs current transforms; a long queue can keep this
        # listener seconds behind the broadcaster under concurrent callbacks.
        self.tf_node = Node("outdoor_execution_tf")
        self.tf_listener = TransformListener(
            self.tf, self.tf_node, spin_thread=True, qos=QoSProfile(depth=1))
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
        self.stop_recovery = None
        self.stop_recovering = False
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
        self.scan, self.scan_at = None, -math.inf
        self.obstacles_off = False
        self.obstacle_scan_pub = self.create_publisher(
            LaserScan, "/outdoor/obstacle_scan", qos_profile_sensor_data
        )
        self.clear_costmaps = [
            self.create_client(Empty, name)
            for name in (
                "/global_costmap/clear_entirely_global_costmap",
                "/local_costmap/clear_entirely_local_costmap",
            )
        ]
        self.gait_schedule = self.gait_task = None
        self.gait_pub = (self.create_publisher(UInt32, self.runtime["gait_output_topic"], 10)
                         if self.runtime.get("gait_switching_enabled", False) else None)
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
            (1.0, self.recover_stop),
            (0.1, self.check_stationary),
            (0.1, self.gait_tick),
            (0.1, self.scan_gate_tick),
        ):
            self.create_timer(
                interval, callback, clock=self.steady, callback_group=self.group
            )
        self.controller_parameters = self.create_client(
            SetParametersAtomically,
            "/controller_server/set_parameters_atomically",
            callback_group=self.group,
        )
        self.http = serve(
            self, port=self.runtime.get("http_port", 19975), host="127.0.0.1"
        )

    def check_stationary(self):
        now = time.monotonic()
        stopped = (
            now - self.velocity_at < self.config.get("localization_timeout_s", 3.0)
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

    async def recover_stop(self):
        # Recover readiness only; the failed task remains failed.
        if (not self.stop_failure or self.stop_recovery is None or self.owner is not None
                or self.stop_recovering or self.clock_rewound):
            return
        self.stop_recovering = True
        try:
            requests, results = self.stop_recovery
            if await self.release_controller(requests, results) and await self.confirm_stop():
                self.stop_failure = None
                self.stop_recovery = None
                self.startup_stopped = True
        finally:
            self.stop_recovering = False

    def gait_tick(self):
        with self.state_lock:
            task = self.tasks.snapshot(self.owner) if self.owner else None
            if (self.gait_pub is None or not task or task["cancel_requested"]
                    or task["state"] not in ("planning", "navigating", "replanning")
                    or not task.get("intervals")):
                self.gait_schedule = self.gait_task = None
                return
            if self.gait_task != task["task_id"]:
                self.gait_schedule = GaitSchedule(task["reference_path"], task["intervals"])
                self.gait_task = task["task_id"]
            position = self.pose()
            if position is None or not 0 <= position["age_s"] < self.config.get("pose_timeout_s", 3.0):
                return
            now, body = time.monotonic(), self.chassis
            motion_age = body.get("motion_feedback_age_s")
            mode_age = body.get("usage_mode_age_s")
            motion_age = motion_age if type(motion_age) in (float, int) else math.inf
            mode_age = mode_age if type(mode_age) in (float, int) else math.inf
            received_age = now - self.chassis_at
            health = self.status(False)["health"]
            eligible = (all(v for k, v in health.items() if k != "chassis")
                        and received_age < 1. and body.get("state") == 17
                        and body.get("usage_mode") == 1 and 0 <= mode_age < 3.
                        and 0 <= motion_age < 1. and body.get("gait_output_enabled") is True)
            command, status = self.gait_schedule.tick(
                position, now, body.get("gait"), max(motion_age, received_age), eligible,
                stopped=(self.stationary_since is not None
                         and now-self.stationary_since >= self.config["stop_hold_s"]
                         and now-self.velocity_at < self.config.get("localization_timeout_s", 3.0)),
                timeout_s=self.config["gait_switch_timeout_s"],
                prepare_timeout_s=self.config.get("gait_prepare_timeout_s", 15.))
            if command is not None:
                self.gait_pub.publish(UInt32(data=command))
            self.tasks.update(task["task_id"], gait_status=status)
            if not status["motion_allowed"]:
                self.command_pub.publish(Twist())

    def controller_id(self):
        return "StairsFollowPath" if self.gait_schedule and self.gait_schedule.target == STAIRS else "FollowPath"

    def motion_allowed(self, task):
        if not task or not task.get("intervals"):
            return True
        state = task.get("gait_status") or {}
        now = time.monotonic()
        age = self.chassis.get("motion_feedback_age_s")
        return (state.get("motion_allowed") is True
                and now-self.chassis_at < 1.
                and type(age) in (float, int) and 0 <= age < 1.
                and self.chassis.get("gait") == state.get("target_code")
                and getattr(self, "active_controller", None) == self.controller_id())

    async def wait_for_ready_pose(self, task_id):
        recovery = (self.geo.get("fusion") or {}).get("recovery_id", 0)
        if recovery != getattr(self, "handled_recovery", 0):
            self.stopping = True
            self.command_pub.publish(Twist())
            requests = getattr(self, "active_follow_requests", [])
            results = getattr(self, "active_follow_results", {})
            if not await self.release_controller(requests, results) or not await self.confirm_stop():
                raise RuntimeError("Cannot release controller after RTK position correction")
            self.handled_recovery = recovery
            self.tasks.update(task_id, state="replanning", reason="RTK restored; replanning after position correction")
        was_stopping = self.stopping
        while True:
            position = await super().wait_for_ready_pose(task_id)
            self.gait_tick()
            task = self.tasks.snapshot(task_id, include_paths=False)
            gait = task.get("gait_status") or {}
            if gait.get("failed"):
                raise RuntimeError("步态切换失败，已停车："+str(gait.get("reason")))
            if not task.get("intervals") or gait.get("motion_allowed"):
                self.stopping = was_stopping
                return position
            self.stopping = True
            self.command_pub.publish(Twist())
            await self.pause(.05)

    def path_obstructed(self, points, position):
        if self.controller_id() == "StairsFollowPath":
            return False
        return super().path_obstructed(points, position)

    def on_geo(self, msg):
        self._state(msg, "geo")
        recovery = (self.geo.get("fusion") or {}).get("recovery_id", 0)
        if self.owner and recovery != getattr(self, "handled_recovery", 0):
            self.stopping = True
            self.command_pub.publish(Twist())

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
        self.scan, self.scan_at = msg, time.monotonic()
        if healthy_scan(
            msg,
            self.get_clock().now().nanoseconds / 1e9,
            self.config["sensor_timeout_s"],
        ):
            self.mark("lidar")

    def stairs_active(self):
        task = self.tasks.snapshot(self.owner, include_paths=False) if self.owner else None
        return stairs_obstacle_off(
            (task or {}).get("gait_status"),
            self.config["stairs_disable_obstacle_layer"],
        )

    def scan_gate_tick(self):
        """Stop marking lidar returns as obstacles while the stair gait runs.

        The sensor topic stays the real scan for the guard and any observer;
        only the obstacle layer's copy is withheld. Existing marks are cleared
        through Nav2's own service instead of publishing a fabricated scan.
        """
        scan = self.scan
        if scan is None:
            return
        stairs = self.stairs_active()
        if stairs != self.obstacles_off:
            self.obstacles_off = stairs
            if stairs:
                for client in self.clear_costmaps:
                    if client.service_is_ready():
                        client.call_async(Empty.Request())
        if stairs:
            return
        if time.monotonic() - self.scan_at >= self.config["sensor_timeout_s"]:
            return
        self.obstacle_scan_pub.publish(scan)

    def status(self, include_task=True):
        now = time.monotonic()
        c = self.config
        g = self.geo
        t = c["sensor_timeout_s"]
        owned = (
            self.tasks.snapshot(self.owner, include_paths=False) if self.owner else None
        )
        health = {
            k: now - self.stats.get(k, -math.inf) < limit
            for k, limit in (("lidar", t), ("global_map", 3), ("local_map", t))
        }
        health.update(
            localization=continuation_allowed(g, owned, c["allow_float_during_fixed_task"])
            and now-self.geo_at < c.get('localization_timeout_s', 3.0),
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
            and now - self.velocity_at < c.get("localization_timeout_s", 3.0),
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
        health["tf"] = pose is not None and pose["age_s"] < c.get("pose_timeout_s", 3.0)
        return {
            "schema_version": 1,
            "ready": all(health.values()),
            "health": health,
            "localization_grade": g.get("grade"),
            "fusion": g.get("fusion"),
            "new_task_ready": g.get("grade") in ("fixed", "float") and all(health.values()),
            "coordinate_reference": g.get("coordinate_reference"),
            "gait_status": owned.get("gait_status") if owned else None,
            "obstacles_from_lidar": not self.stairs_active(),
            "gait_switching_enabled": self.runtime.get("gait_switching_enabled", False),
            "local_avoidance_enabled": not self.owner or self.controller_id() == "FollowPath",
            "heading": g.get("heading"),
            "feedback": g.get("feedback"),
            "pose": pose,
            "command_topic": self.runtime["velocity_output_topic"],
            "not_ready_reasons": [k for k, v in health.items() if not v],
            "measured_velocity": list(self.velocity)
            if all(math.isfinite(v) for v in self.velocity)
            else None,
            "velocity_fresh": now - self.velocity_at < c.get("localization_timeout_s", 3.0),
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
            and self.geo.get("grade") in ("fixed", "float")
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

    async def wait_for_arrival_confirmation(self, task_id):
        deadline = time.monotonic()+self.config["navigation_recovery_timeout_s"]
        while not arrival_confirmed(self.geo):
            if self.interrupted(task_id):
                raise InterruptedError("Canceled while waiting for RTK at destination")
            self.stopping = True
            self.command_pub.publish(Twist())
            self.tasks.update(task_id, state="planning", reason="Waiting for RTK to confirm arrival")
            if time.monotonic() >= deadline:
                raise RuntimeError("RTK did not recover to confirm arrival; stopped")
            await self.pause(.1)

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
        if self.controller_id() != "StairsFollowPath":
            path = await super().guided_path(guidance, window, position, task_id)
        else:
            path = self.stairs_path(guidance, window, position)
        goal = self.tasks.snapshot(task_id)["goal"]
        endpoint = path.poses[-1].pose.position if path and path.poses else None
        reaches_final = (endpoint is not None and math.hypot(
            endpoint.x-guidance.points[-1]['x'], endpoint.y-guidance.points[-1]['y'])
            <= self.config.get('planner_tolerance_m', .2)+.01)
        if reaches_final and "yaw" in goal:
            q = path.poses[-1].pose.orientation
            q.z, q.w = math.sin(goal["yaw"]/2), math.cos(goal["yaw"]/2)
        return path

    def stairs_path(self, guidance, window, position):
        # Stairs follow the measured reference; NavFn must not detour around steps.
        path = NavPath()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        vertices = [position, *(guidance.points[i] for i in window)]
        points = [position]
        for a, b in zip(vertices, vertices[1:]):
            steps = max(1, math.ceil(math.hypot(b['x']-a['x'], b['y']-a['y'])/.1))
            points.extend({'x':a['x']+(b['x']-a['x'])*i/steps,
                           'y':a['y']+(b['y']-a['y'])*i/steps} for i in range(1, steps+1))
        for i, point in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(point["x"]), float(point["y"])
            a, b = (point, points[i+1]) if i+1 < len(points) else (points[i-1], point)
            yaw = math.atan2(b["y"]-a["y"], b["x"]-a["x"])
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            path.poses.append(pose)
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
        node.tf_listener.executor.shutdown()
        node.tf_listener.dedicated_listener_thread.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
