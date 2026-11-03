"""Single task owner: NavigateToPose, sensor-only Nav2 and reference-route HTTP API."""

import math
import os
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from unique_identifier_msgs.msg import UUID

from outdoornav.api import ApiError, serve
from outdoornav.bundle import is_free, load_bundle, masks
from outdoornav.routing import RouteGuidance, validate_point
from outdoornav.tasks import TaskStore, TERMINAL, ProgressBudget
from outdoornav.outdoor import OutdoorService
from .maps import LATCHED, Maps
from .obstacles import Obstacles
from .motion import MotionPrediction


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def length(points):
    return sum(
        math.dist((a["x"], a["y"]), (b["x"], b["y"]))
        for a, b in zip(points, points[1:])
    )


def lateral_difference(a, b):
    """Maximum new-path distance to the old polyline, rather than old vertices."""
    if not b:
        return math.inf
    old = np.array([(p["x"], p["y"]) for p in b])
    starts, vectors = old[:-1], old[1:] - old[:-1]
    denom = np.maximum((vectors * vectors).sum(axis=1), 1e-12)
    distances = []
    for point in a[:: max(1, len(a) // 100)]:
        delta = np.array([point["x"], point["y"]]) - starts
        t = np.clip((delta * vectors).sum(axis=1) / denom, 0, 1)
        distances.append(
            np.sqrt(((delta - t[:, None] * vectors) ** 2).sum(axis=1)).min()
        )
    return max(distances, default=0)


class Navigation(Node):
    def __init__(self, hardware=False):
        super().__init__("navigation")
        self.hardware = hardware
        self.base_frame = "base_link" if hardware else "base_footprint"
        self.arrival_tolerance = 0.50 if hardware else 0.25
        self.declare_parameter("bundle", "")
        self.bundle_path = self.get_parameter("bundle").value
        self.bundle = load_bundle(self.bundle_path)
        # Scene geometry validates API goals and fixture placement only; it is not
        # an occupancy prior for either Nav2 costmap.
        self.free = None if hardware else masks(self.bundle_path)
        self.tasks = TaskStore(
            self.bundle["scene_version"], self.bundle["route_version"]
        )
        self.group = ReentrantCallbackGroup()
        self.declare_parameter("run_dir", "")
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.maps = Maps(self, self.bundle)
        self.stats, self.lifecycle, self.lifecycle_pending = {}, {}, set()
        self.clock_value, self.clock_changed = None, 0.0
        self.clock_rewound = False
        self.localization = "no_fix"
        self.global_map, self.local_map = None, None
        self.costmap_view_cache = {}
        self.raw_command, self.raw_received = Twist(), 0.0
        self.velocity, self.velocity_at = (math.inf, math.inf), 0.0
        self.owner, self.stopping = None, True
        self.stop_failure = None
        self.handles, self.requests, self.cancel_sent = {}, {}, set()
        self.state_lock = threading.RLock()
        self.last_sim_time = 0.0
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
            name: self.create_client(
                GetState, f"/{name}/get_state", callback_group=self.group
            )
            for name in ("planner_server", "controller_server")
        }
        self.command_pub = self.create_publisher(
            Twist, "/outdoor/safe_command" if hardware else "/cmd_vel", 10
        )
        self.path_pub = self.create_publisher(
            Path, "/navigation/execution_path", LATCHED
        )
        from rclpy.qos import qos_profile_sensor_data

        for message_type, topic, key in ([] if hardware else [
            (Image, "/rtk/camera/depth_image", "depth"),
            (PointCloud2, "/rtk/camera/points", "points"),
            (LaserScan, "/rtk/scan", "lidar"),
        ]):
            self.create_subscription(
                message_type,
                topic,
                lambda msg, key=key: self.mark(key),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            ClockMessage, "/clock", self.on_clock, qos_profile_sensor_data
        )
        self.create_subscription(
            String, "/rtk/localization_status", self.on_localization, LATCHED
        )
        # Status is published on transitions; freshness comes from localization output.
        self.create_subscription(
            Odometry, "/odometry/global", lambda msg: self.mark("localization"), 10
        )
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.on_global, LATCHED
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.on_local, LATCHED
        )
        if not hardware:
            self.create_subscription(Odometry, "/odometry/local", self.on_odom, 10)
        self.create_subscription(
            Twist, "/navigation/controller_command", self.on_command, 10
        )
        self.create_subscription(PoseStamped, "/goal_pose", self.on_rviz_goal, 1)
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
        self.create_timer(
            0.05, self.safety_tick, clock=self.steady, callback_group=self.group
        )
        self.create_timer(
            0.1, self.dispatch, clock=self.steady, callback_group=self.group
        )
        self.create_timer(
            1.0, self.poll_lifecycle, clock=self.steady, callback_group=self.group
        )
        self.obstacles = None
        if not hardware:
            self.obstacles = Obstacles(self, self.get_parameter("run_dir").value)
            self.prediction = MotionPrediction(self)
        self.outdoor = OutdoorService(self.bundle, None, self.tasks, self.status)
        self.http = None if hardware else serve(self)

    def mark(self, key):
        with self.state_lock:
            self.stats[key] = time.monotonic()

    def on_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec / 1e9
        with self.state_lock:
            if value != self.clock_value:
                self.clock_changed = time.monotonic()
            if self.clock_value is not None and value < self.clock_value:
                self.clock_rewound = True
                if self.owner:
                    self.tasks.update(
                        self.owner,
                        state="canceling",
                        cancel_requested=True,
                        reason="Simulation clock moved backward",
                    )
                self.stats.clear()
                self.stopping = True
                self.stop_failure = "Simulation clock moved backward; restart required"
            self.clock_value = value

    def on_localization(self, msg):
        self.localization = msg.data

    def on_global(self, msg):
        self.global_map = msg
        self.mark("global_map")

    def on_local(self, msg):
        self.local_map = msg
        self.mark("local_map")

    def costmap_snapshot(self, layer):
        from outdoornav.api import ApiError
        from outdoornav.costmap_view import grid_snapshot
        if layer not in ('local', 'global'):
            raise ApiError(404, 'Unknown costmap layer')
        with self.state_lock:
            message = self.local_map if layer == 'local' else self.global_map
            received = self.stats.get(layer + '_map')
        if message is None or received is None:
            raise ApiError(503, 'Waiting for ' + layer + ' costmap')
        frame = message.header.frame_id
        robot, path = None, []
        try:
            transform = self.tf.lookup_transform(frame, self.base_frame, rclpy.time.Time())
            t, q = transform.transform.translation, transform.transform.rotation
            robot = {'x': t.x, 'y': t.y, 'yaw': yaw(q),
                     'age_s': max(0., self.get_clock().now().nanoseconds/1e9 -
                                  transform.header.stamp.sec - transform.header.stamp.nanosec/1e9)}
        except Exception:
            pass  # No TF is displayed as unavailable, never as a fabricated pose.
        try:
            task = self.tasks.snapshot() or {}
            points = task.get('execution_path') or []
            if frame == 'map':
                path = points
            elif points:
                transform = self.tf.lookup_transform(frame, 'map', rclpy.time.Time())
                t, angle = transform.transform.translation, yaw(transform.transform.rotation)
                c, s = math.cos(angle), math.sin(angle)
                path = [{'x': t.x+c*p['x']-s*p['y'], 'y': t.y+s*p['x']+c*p['y']} for p in points]
        except Exception:
            pass
        # Encode once per grid message, not once per viewer poll. No disk writes.
        map_from_frame = {'x': 0., 'y': 0., 'yaw': 0., 'age_s': 0.} if frame == 'map' else None
        if map_from_frame is None:
            try:
                transform = self.tf.lookup_transform('map', frame, rclpy.time.Time())
                t = transform.transform.translation
                map_from_frame = {'x': t.x, 'y': t.y, 'yaw': yaw(transform.transform.rotation),
                    'age_s': max(0., self.get_clock().now().nanoseconds/1e9 -
                                 transform.header.stamp.sec-transform.header.stamp.nanosec/1e9)}
            except Exception:
                pass
        cache = self.costmap_view_cache.get(layer)
        if cache is None or cache[0] is not message:
            cache = (message, grid_snapshot(message, 0., theme=layer))
            self.costmap_view_cache[layer] = cache
        source_age = self.get_clock().now().nanoseconds/1e9 - cache[1]['stamp_s']
        age = max(0., time.monotonic()-received, source_age)
        # Image, overlays and metadata use the grid's own frame (local is odom).
        return {**cache[1], 'age_s': age, 'stale': age >= 3., 'robot': robot, 'path': path,
                'map_from_frame': map_from_frame}

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        t = msg.twist.twist
        if not -0.1 <= age < (self.config["sensor_timeout_s"] if self.hardware else 1) or not all(math.isfinite(v) for v in (t.linear.x, t.linear.y, t.angular.z)):
            return
        self.velocity = (math.hypot(t.linear.x, t.linear.y), abs(t.angular.z))
        self.velocity_at = time.monotonic() - max(0, age)

    def on_command(self, msg):
        self.raw_command, self.raw_received = msg, time.monotonic()

    def poll_lifecycle(self):
        for name, client in self.lifecycle_clients.items():
            if name in self.lifecycle_pending or not client.service_is_ready():
                continue
            self.lifecycle_pending.add(name)
            future = client.call_async(GetState.Request())

            def done(future, name=name):
                self.lifecycle_pending.discard(name)
                try:
                    self.lifecycle[name] = (
                        future.result().current_state.id == 3,
                        time.monotonic(),
                    )
                except Exception:
                    self.lifecycle[name] = (False, time.monotonic())

            future.add_done_callback(done)

    def pose(self):
        try:
            transform = self.tf.lookup_transform(
                "map", self.base_frame, rclpy.time.Time()
            )
        except Exception:
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if age < -0.1:
            return None
        return {
            "frame_id": "map",
            "x": t.x,
            "y": t.y,
            "yaw": yaw(q),
            "stamp_s": stamp,
            "age_s": max(0, age),
        }

    def status(self, include_task=True):
        now = time.monotonic()
        with self.state_lock:
            health = {
                key: now - self.stats.get(key, -math.inf) < 3.0
                for key in (
                    "depth",
                    "points",
                    "lidar",
                    "localization",
                    "global_map",
                    "local_map",
                )
            }
            health["localization"] &= self.localization == "ok"
            health["obstacle_services"] = all(
                client.service_is_ready()
                for client in (
                    self.obstacles.create,
                    self.obstacles.remove,
                    self.obstacles.move,
                )
            )
            health["clock"] = (
                self.clock_value is not None and now - self.clock_changed < 3.0
            )
            health.update(
                {
                    name: self.lifecycle.get(name, (False, 0))[0]
                    and now - self.lifecycle.get(name, (False, 0))[1] < 5
                    for name in self.lifecycle_clients
                }
            )
        pose = self.pose()
        health["tf"] = pose is not None and pose["age_s"] < 1.0
        health["controller_stopped"] = self.stop_failure is None
        health["actions"] = (
            self.path_client.server_is_ready()
            and self.follow_client.server_is_ready()
            and self.navigate_client.server_is_ready()
        )
        return {
            "schema_version": 1,
            "run_id": os.environ.get("OUTDOORNAV_RUN_ID"),
            "scene_version": self.bundle["scene_version"],
            "route_version": self.bundle["route_version"],
            "ready": all(health.values()),
            "health": health,
            "localization": self.localization,
            "pose": pose,
            "task": self.tasks.snapshot() if include_task else None,
        }

    def map_snapshot(self):
        return self.bundle

    def validated(self, payload):
        goal, blocked, route = self.outdoor.validated(payload)
        if not self.hardware and not self.outdoor.preference and not is_free(self.free, self.bundle['grid'], goal):
            raise ValueError('Destination is not in traversable scene space')
        return goal, blocked, route

    def preview(self, payload):
        return self.outdoor.preview(payload)

    def submit(self, payload):
        return self.outdoor.submit(payload)

    def on_rviz_goal(self, msg):
        try:
            self.submit(
                {
                    "frame_id": msg.header.frame_id,
                    "scene_version": self.bundle["scene_version"],
                    "route_version": self.bundle["route_version"],
                    "goal": {"x": msg.pose.position.x, "y": msg.pose.position.y},
                }
            )
        except ValueError as error:
            self.get_logger().warn(str(error))

    def on_goal(self, request):
        try:
            self.validated(
                {
                    "frame_id": request.pose.header.frame_id,
                    "scene_version": self.bundle["scene_version"],
                    "route_version": self.bundle["route_version"],
                    "goal": {
                        "x": request.pose.pose.position.x,
                        "y": request.pose.pose.position.y,
                    },
                }
            )
            return GoalResponse.ACCEPT
        except ValueError:
            return GoalResponse.REJECT

    def on_accepted(self, handle):
        task_id = bytes(handle.goal_id.uuid).hex()
        try:
            self.tasks.snapshot(task_id, include_paths=False)
        except KeyError:
            p = handle.request.pose.pose.position
            self.tasks.submit({"x": p.x, "y": p.y}, task_id=task_id)
        self.handles[task_id] = handle
        self.requests.setdefault(task_id, None)
        handle.execute()

    def on_cancel(self, handle):
        self.tasks.cancel(bytes(handle.goal_id.uuid).hex())
        return CancelResponse.ACCEPT

    def dispatch(self):
        with self.tasks.lock:
            for task_id, task in self.tasks.tasks.items():
                if task.state == "canceling" and task_id not in self.requests:
                    self.tasks.update(
                        task_id, state="canceled", reason="Canceled before dispatch"
                    )
        current = self.tasks.snapshot(include_paths=False)
        if (
            current
            and current["state"] == "accepted"
            and current["task_id"] not in self.requests
        ):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = (
                current["goal"]["x"],
                current["goal"]["y"],
            )
            goal.pose.pose.orientation.w = 1.0
            uid = UUID(uuid=list(bytes.fromhex(current["task_id"])))
            future = self.navigate_client.send_goal_async(goal, goal_uuid=uid)
            self.requests[current["task_id"]] = future

            def sent(future, task_id=current["task_id"]):
                try:
                    if not future.result().accepted:
                        self.tasks.update(
                            task_id,
                            state="failed",
                            reason="Navigation action rejected the goal",
                        )
                except Exception as error:
                    self.tasks.update(task_id, state="failed", reason=str(error))

            future.add_done_callback(sent)
        for task_id, handle in list(self.handles.items()):
            try:
                state = self.tasks.snapshot(task_id, include_paths=False)
            except KeyError:
                self.handles.pop(task_id, None)
                continue
            if (
                state["cancel_requested"]
                and state["state"] not in TERMINAL
                and task_id not in self.cancel_sent
            ):
                request = CancelGoal.Request()
                request.goal_info.goal_id = handle.goal_id
                self.cancel_client.call_async(request)
                self.cancel_sent.add(task_id)

    def safety_tick(self):
        state = (
            self.tasks.snapshot(self.owner, include_paths=False) if self.owner else None
        )
        safe = (
            not self.stopping
            and state
            and not state["cancel_requested"]
            and state["state"] in ("navigating", "replanning")
            and self.status(False)["ready"]
            and time.monotonic() - self.raw_received < (
                self.config["command_timeout_s"] if self.hardware else 0.5
            )
        )
        self.command_pub.publish(self.raw_command if safe else Twist())

    def path_obstructed(self, points, position):
        if not points or self.global_map is None:
            return False
        message = self.global_map
        meta = message.info
        closest = min(
            range(len(points)),
            key=lambda i: math.hypot(
                points[i]["x"] - position["x"], points[i]["y"] - position["y"]
            ),
        )
        for point in points[closest:]:
            if math.hypot(point["x"] - position["x"], point["y"] - position["y"]) > 8:
                break
            col = int(
                math.floor((point["x"] - meta.origin.position.x) / meta.resolution)
            )
            row = int(
                math.floor((point["y"] - meta.origin.position.y) / meta.resolution)
            )
            if (
                0 <= row < meta.height
                and 0 <= col < meta.width
                and message.data[row * meta.width + col] >= 70
            ):
                return True
        return False

    async def pause(self, seconds=0.05):
        future = Future()

        def finish():
            timer.cancel()
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(
            seconds, finish, clock=self.steady, callback_group=self.group
        )
        try:
            await future
        finally:
            self.destroy_timer(timer)

    def interrupted(self, task_id):
        try:
            state = self.tasks.snapshot(task_id, include_paths=False)
            return state["cancel_requested"] or state["state"] in TERMINAL
        except KeyError:
            return True

    async def wait_for_ready_pose(self, task_id):
        """Pause the task briefly; never keep sending motion during lost feedback."""
        timeout = getattr(self, 'config', {}).get('navigation_recovery_timeout_s', 5.0)
        deadline = time.monotonic() + timeout
        was_stopping = self.stopping
        while True:
            if self.interrupted(task_id):
                raise InterruptedError('Task canceled or superseded')
            status = self.status(False)
            position = self.pose()
            if (status['ready'] and position is not None
                    and all(math.isfinite(position[k]) for k in ('x', 'y', 'yaw'))
                    and 0 <= position.get('age_s', math.inf) < getattr(self, 'config', {}).get('sensor_timeout_s', 1.0)):
                self.stopping = was_stopping
                return position
            self.stopping = True
            self.command_pub.publish(Twist())
            health = status.get('health', {})
            if getattr(self, 'clock_rewound', False) or any(health.get(key) is False for key in (
                    'exclusive_owners', 'motion_enabled', 'production_mode', 'service_active')):
                raise RuntimeError('Navigation disabled or ownership/clock conflict; stopped')
            if time.monotonic() >= deadline:
                raise RuntimeError('Position or navigation unavailable after %.1f s; stopped' % timeout)
            await self.pause(0.1)

    async def result(self, future, task_id, timeout=10):
        began = self.get_clock().now().nanoseconds / 1e9
        while not future.done():
            if self.interrupted(task_id):
                raise InterruptedError("Task canceled or superseded")
            if not self.status(False)["ready"]:
                await self.wait_for_ready_pose(task_id)
            if self.get_clock().now().nanoseconds / 1e9 - began > timeout:
                raise RuntimeError("Nav2 action timed out")
            await self.pause()
        if self.interrupted(task_id):
            raise InterruptedError("Task canceled or superseded")
        return future.result()

    async def confirm_stop(self):
        self.stopping = True
        deadline, stable = time.monotonic() + 15, None
        while time.monotonic() < deadline:
            now = time.monotonic()
            stopped = (
                time.monotonic() - self.velocity_at < (self.config["sensor_timeout_s"] if self.hardware else 1)
                and self.velocity[0] <= (self.config.get('stopped_linear_mps', .01) if self.hardware else .01)
                and self.velocity[1] <= (self.config.get('stopped_angular_rps', .01) if self.hardware else .01)
            )
            stable = (now if stable is None else stable) if stopped else None
            if stable is not None and now - stable >= (self.config.get('stop_hold_s', 1.) if self.hardware else 1.):
                return True
            await self.pause()
        return False

    async def release_controller(self, requests, results):
        """Reuse original result futures, canceling only goals still in flight.

        Humble's get_result_async() sends a NEW service request on every call.
        Requerying every old, preempted goal at arrival can exhaust the shared
        cleanup deadline even though those goals already returned terminal results.
        """
        deadline = time.monotonic() + 5
        for pending in requests:
            while not pending.done() and time.monotonic() < deadline:
                await self.pause()
            if not pending.done():
                return False
            try:
                follower = pending.result()
                if not follower.accepted:
                    continue
                result = results.get(pending)
                if result is None:
                    # An acceptance may arrive after execute() was interrupted.
                    result = follower.get_result_async()
                    results[pending] = result
                if not result.done():
                    follower.cancel_goal_async()
                    while not result.done() and time.monotonic() < deadline:
                        await self.pause()
                if not result.done() or result.result().status not in (
                    GoalStatus.STATUS_SUCCEEDED,
                    GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_ABORTED,
                ):
                    return False
            except Exception:
                return False
        return True

    async def guided_path(self, guidance, window, position, task_id):
        """Join sensor-costmap NavFn legs into one continuous RPP path.

        Explicit starts prevent every leg from being planned from the robot.
        Preserve the planner's obstacle detours; do not substitute road lines or
        write the reference geometry into either costmap.
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        start = validate_point(position)
        for offset, index in enumerate(window):
            point = guidance.points[index]
            request = ComputePathToPose.Goal()
            request.goal.header = path.header
            request.goal.pose.position.x = point["x"]
            request.goal.pose.position.y = point["y"]
            previous = guidance.points[max(0, index - 1)]
            heading = math.atan2(point["y"] - previous["y"], point["x"] - previous["x"])
            request.goal.pose.orientation.z = math.sin(heading / 2)
            request.goal.pose.orientation.w = math.cos(heading / 2)
            request.use_start = True
            request.start.header = path.header
            request.start.pose.position.x = start["x"]
            request.start.pose.position.y = start["y"]
            request.start.pose.orientation.w = 1.0
            planner = await self.result(self.path_client.send_goal_async(request), task_id)
            if not planner.accepted:
                raise RuntimeError("Planner rejected the goal")
            try:
                result = await self.result(planner.get_result_async(), task_id)
            except Exception:
                planner.cancel_goal_async()
                raise
            leg = result.result.path.poses
            if self.hardware and guidance.final_leg and len(leg) == 1:
                # A coincident destination still needs a path for final heading alignment.
                leg = [request.start, *leg]
            if result.status != GoalStatus.STATUS_SUCCEEDED or len(leg) < 2:
                # A sampled station may itself be occupied. Try the next station
                # on the SAME straight edge, allowing NavFn to go around it.
                # Never skip a bend or the window/final goal to obtain a shortcut.
                if index not in guidance.bends and offset + 1 < len(window):
                    next_point = guidance.points[window[offset + 1]]
                    if math.hypot(next_point["x"] - start["x"], next_point["y"] - start["y"]) <= 4.1:
                        continue
                return None
            # NavFn tolerance can end a leg near (not exactly on) a via point.
            # Start the next leg at its ACTUAL end to avoid a disconnected join.
            if path.poses:
                path.poses.extend(leg[1:])
            else:
                path.poses.extend(leg)
            end = path.poses[-1].pose.position
            start = {"x": end.x, "y": end.y}
        # Intermediate leg-end orientations must not make the controller stop.
        for a, b in zip(path.poses, path.poses[1:]):
            heading = math.atan2(b.pose.position.y - a.pose.position.y, b.pose.position.x - a.pose.position.x)
            a.pose.orientation.z, a.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
        return path

    async def prepare_route(self, route, task_id):
        pass

    def arrival_heading_ok(self, position, goal):
        return True

    async def execute(self, handle):
        task_id = bytes(handle.goal_id.uuid).hex()
        follow_requests, follow_results = [], {}
        outcome, reason = "failed", None
        stop_confirmed, completion_pose = False, None
        try:
            # A successor cannot command while its predecessor is stopping.
            while True:
                if self.stop_failure:
                    raise RuntimeError(self.stop_failure)
                if self.interrupted(task_id):
                    raise InterruptedError("Task canceled before execution")
                with self.state_lock:
                    if self.owner is None:
                        self.owner = task_id
                        self.stopping = True
                        break
                await self.pause()
            state = self.tasks.snapshot(task_id, include_paths=False)
            if self.interrupted(task_id):
                raise InterruptedError("Task canceled before planning")
            self.tasks.update(task_id, state="planning")
            position = await self.wait_for_ready_pose(task_id)
            # Preview and execution use exactly the same road-preference policy.
            route = self.outdoor.plan_from(position, state["goal"], state["blocked_road_ids"],
                                           state.get("road_offroad_cost"))
            await self.prepare_route(route, task_id)
            tolerance = state.get("arrival_tolerance_m") or self.arrival_tolerance
            goal_point = state["goal"]
            position = await self.wait_for_ready_pose(task_id)
            guidance = RouteGuidance(route["reference_path"], position, goal_point)
            self.tasks.update(task_id, reference_path=route["reference_path"])
            current_points, follow_result = [], None
            sent_window = None
            last_plan, last_send = -math.inf, -math.inf
            policy = self.config if self.hardware else {}
            budget = ProgressBudget(planner_patience_s=policy.get('planner_patience_s', 0.),
                                    controller_patience_s=policy.get('controller_patience_s', 20.))
            started_sim = self.get_clock().now().nanoseconds / 1e9
            while True:
                if self.interrupted(task_id):
                    raise InterruptedError("Task canceled or superseded")
                position = await self.wait_for_ready_pose(task_id)
                now = self.get_clock().now().nanoseconds / 1e9
                budget.moved(position["x"], position["y"], now)
                window = guidance.window(position)
                guidance_changed = window != sent_window
                remaining = math.dist(
                    (position["x"], position["y"]), (goal_point["x"], goal_point["y"])
                )
                route_remaining = length([position, *guidance.points[guidance.index:]])
                self.tasks.update(task_id, distance_remaining_m=route_remaining)
                feedback = NavigateToPose.Feedback()
                feedback.current_pose.header.frame_id = "map"
                feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
                (
                    feedback.current_pose.pose.position.x,
                    feedback.current_pose.pose.position.y,
                ) = position["x"], position["y"]
                (
                    feedback.current_pose.pose.orientation.z,
                    feedback.current_pose.pose.orientation.w,
                ) = math.sin(position["yaw"] / 2), math.cos(position["yaw"] / 2)
                feedback.distance_remaining = route_remaining
                handle.publish_feedback(feedback)
                # Task arrival is not the controller's tight per-window goal.
                # Require ordered route distance too, so a nearby goal across a
                # U-shaped road cannot shortcut unvisited bends. Cancel/release
                # FollowPath and verify measured stop before success below.
                if (remaining <= tolerance and route_remaining <= tolerance
                        and self.arrival_heading_ok(position, goal_point)):
                    self.stopping = True
                    if not await self.release_controller(follow_requests, follow_results):
                        raise RuntimeError("Could not release controller at arrival")
                    if not await self.confirm_stop():
                        raise RuntimeError("Could not confirm robot stop at arrival")
                    if self.interrupted(task_id):
                        raise InterruptedError("Task canceled while stopping")
                    if not self.status(False)["ready"]:
                        raise RuntimeError("Localization or sensors lost at arrival")
                    stopped_pose = await self.wait_for_ready_pose(task_id)
                    remaining = math.dist(
                        (stopped_pose["x"], stopped_pose["y"]),
                        (goal_point["x"], goal_point["y"]),
                    )
                    self.tasks.update(task_id, distance_remaining_m=remaining)
                    if remaining <= tolerance and self.arrival_heading_ok(stopped_pose, goal_point):
                        outcome, stop_confirmed = "succeeded", True
                        completion_pose = stopped_pose
                        break
                    # The estimate can move while braking. Resume planning rather
                    # than reporting arrival outside the unchanged tolerance.
                    current_points, follow_result = [], None
                if now - started_sim > 1800:
                    raise RuntimeError(
                        "Navigation exceeded the 1800 simulation-second task limit"
                    )
                if follow_result is not None and follow_result.done():
                    ended = follow_result.result().status
                    current_points, follow_result = [], None
                    # Only the CURRENT action can spend the failure budget. Old
                    # preempted futures are deliberately not polled. Reaching a
                    # temporary horizon is not a controller failure either.
                    if ended != GoalStatus.STATUS_SUCCEEDED and not budget.controller_ended(now):
                        raise RuntimeError(
                            "Controller could not make progress on this road"
                        )
                if not current_points or guidance_changed or now - last_plan >= 1.0:
                    last_plan = now
                    path = await self.guided_path(guidance, window, position, task_id)
                    points = [
                        {"x": p.pose.position.x, "y": p.pose.position.y}
                        for p in path.poses
                    ] if path is not None else []
                    if len(points) < 2:
                        if not budget.planned(False, now):
                            raise RuntimeError(
                                "No traversable path in the sensor costmap"
                            )
                        self.stopping = True
                        if not await self.release_controller(follow_requests, follow_results):
                            raise RuntimeError('Cannot release controller while waiting for a path')
                        current_points, follow_result = [], None
                        self.tasks.update(task_id, state='planning', reason='暂时无可行路径，停车等待并重规划')
                        await self.pause(0.5)
                        continue
                    budget.planned(True, now)
                    if not guidance_changed and current_points and follow_result and not follow_result.done():
                        if not self.path_obstructed(current_points, position) and (
                            now - last_send < 6
                            or (
                                lateral_difference(points, current_points) < 0.45
                                and abs(length(points) - length(current_points)) < 1.2
                            )
                        ):
                            await self.pause(0.1)
                            continue
                    self.tasks.update(
                        task_id, state="replanning" if current_points else "planning"
                    )
                    command = FollowPath.Goal()
                    command.path = path
                    command.controller_id = "FollowPath"
                    command.goal_checker_id = "goal_checker"
                    pending = self.follow_client.send_goal_async(command)
                    follow_requests.append(pending)

                    def accepted(future):
                        # Even a response arriving after cancellation must be stopped.
                        try:
                            follower = future.result()
                            if self.interrupted(task_id) and follower.accepted:
                                follower.cancel_goal_async()
                        except Exception:
                            pass

                    pending.add_done_callback(accepted)
                    follower = await self.result(pending, task_id)
                    if not follower.accepted:
                        raise RuntimeError("Controller rejected the planned path")
                    follow_result = follower.get_result_async()
                    follow_results[pending] = follow_result
                    current_points, last_send = points, now
                    sent_window = window
                    revision = (
                        self.tasks.snapshot(task_id, include_paths=False)[
                            "path_revision"
                        ]
                        + 1
                    )
                    self.tasks.update(
                        task_id,
                        state="navigating",
                        path_revision=revision,
                        execution_path=points,
                        reason=None,
                    )
                    self.stopping = False
                    self.path_pub.publish(path)
                await self.pause(0.1)
        except InterruptedError as error:
            outcome, reason = "canceled", str(error)
            if (
                self.tasks.snapshot(task_id, include_paths=False)["reason"]
                == "Simulation clock moved backward"
            ):
                outcome, reason = "failed", "Simulation clock moved backward"
        except Exception as error:
            reason = str(error)
            self.tasks.update(task_id, state="canceling", reason=reason)
            self.get_logger().error(f"Task {task_id}: {reason}")
        finally:
            if self.owner == task_id:
                if self.clock_rewound:
                    outcome, reason = "failed", "Simulation clock moved backward"
                self.stopping = True
                released = await self.release_controller(follow_requests, follow_results)
                stopped = stop_confirmed or await self.confirm_stop()
                if not released or not stopped:
                    self.stop_failure = (
                        ((reason + "; ") if reason else "")
                        + "Could not confirm controller release and robot stop; restart required"
                    )
                    outcome, reason = "failed", self.stop_failure
                self.tasks.update(
                    task_id,
                    state=outcome,
                    reason=reason,
                    completed_sim_s=self.get_clock().now().nanoseconds / 1e9,
                    completion_pose=completion_pose or self.pose(),
                )
                with self.state_lock:
                    self.owner = None
            else:
                self.tasks.update(
                    task_id,
                    state=outcome,
                    reason=reason,
                    completed_sim_s=self.get_clock().now().nanoseconds / 1e9,
                    completion_pose=self.pose(),
                )
            self.handles.pop(task_id, None)
            self.requests.pop(task_id, None)
            self.cancel_sent.discard(task_id)
            if handle.is_cancel_requested:
                handle.canceled()
            elif outcome == "succeeded":
                handle.succeed()
            else:
                handle.abort()
        return NavigateToPose.Result()

    def destroy_node(self):
        if rclpy.ok():
            self.command_pub.publish(Twist())
        if self.http is not None:
            self.http.shutdown()
            self.http.server_close()
        if self.obstacles is not None:
            self.obstacles.close()
        self.server.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = Navigation()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
