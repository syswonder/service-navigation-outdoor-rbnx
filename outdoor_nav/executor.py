"""Reference-route execution, extracted from outdoorNav without road planning."""

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

from .http import ApiError, serve
from .core.guidance import RouteGuidance, validate_point, free_via_point, observed_obstacle
from .core.tasks import TaskStore, TERMINAL, ProgressBudget
from .core.scan_health import healthy_scan
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from pathlib import Path as FilePath
import json
from .configuration import load_runtime

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


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


class Executor(Node):
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

    def on_global(self, msg):
        self.global_map = msg
        self.mark("global_map")

    def on_local(self, msg):
        self.local_map = msg
        self.mark("local_map")

    def costmap_snapshot(self, layer):
        from .http import ApiError
        from .core.costmap_view import grid_snapshot

        if layer not in ("local", "global"):
            raise ApiError(404, "Unknown costmap layer")
        with self.state_lock:
            message = self.local_map if layer == "local" else self.global_map
            received = self.stats.get(layer + "_map")
        if message is None or received is None:
            raise ApiError(503, "Waiting for " + layer + " costmap")
        frame = message.header.frame_id
        robot, path = None, []
        overlay_warnings = []
        def transform_fresh(transform):
            stamp = transform.header.stamp
            age = self.get_clock().now().nanoseconds/1e9-stamp.sec-stamp.nanosec/1e9
            return -.1 <= age < 3.
        try:
            transform = self.tf.lookup_transform(
                frame, self.base_frame, rclpy.time.Time()
            )
            if not transform_fresh(transform):
                raise ValueError('Robot transform is stale')
            t, q = transform.transform.translation, transform.transform.rotation
            robot = {
                "x": t.x,
                "y": t.y,
                "yaw": yaw(q),
                "age_s": max(
                    0.0,
                    self.get_clock().now().nanoseconds / 1e9
                    - transform.header.stamp.sec
                    - transform.header.stamp.nanosec / 1e9,
                ),
            }
        except Exception:
            overlay_warnings.append('robot_transform_unavailable')
        try:
            task = self.tasks.snapshot() or {}
            points = task.get("execution_path") or []
            if frame == "map":
                path = points
            elif points:
                transform = self.tf.lookup_transform(frame, "map", rclpy.time.Time())
                if not transform_fresh(transform):
                    raise ValueError('Path transform is stale')
                t, angle = (
                    transform.transform.translation,
                    yaw(transform.transform.rotation),
                )
                c, s = math.cos(angle), math.sin(angle)
                path = [
                    {
                        "x": t.x + c * p["x"] - s * p["y"],
                        "y": t.y + s * p["x"] + c * p["y"],
                    }
                    for p in points
                ]
        except Exception:
            path = []
            overlay_warnings.append('path_transform_unavailable')
        # Encode once per grid message, not once per viewer poll. No disk writes.
        map_from_frame = (
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "age_s": 0.0} if frame == "map" else None
        )
        if map_from_frame is None:
            try:
                transform = self.tf.lookup_transform("map", frame, rclpy.time.Time())
                t = transform.transform.translation
                map_from_frame = {
                    "x": t.x,
                    "y": t.y,
                    "yaw": yaw(transform.transform.rotation),
                    "age_s": max(
                        0.0,
                        self.get_clock().now().nanoseconds / 1e9
                        - transform.header.stamp.sec
                        - transform.header.stamp.nanosec / 1e9,
                    ),
                }
            except Exception:
                pass
        cache = self.costmap_view_cache.get(layer)
        if cache is None or cache[0] is not message:
            cache = (message, grid_snapshot(message, 0.0, theme=layer))
            self.costmap_view_cache[layer] = cache
        source_age = self.get_clock().now().nanoseconds / 1e9 - cache[1]["stamp_s"]
        age = max(0.0, time.monotonic() - received, source_age)
        # Image, overlays and metadata use the grid's own frame (local is odom).
        return {
            **cache[1],
            "age_s": age,
            "stale": age >= 3.0,
            "robot": robot,
            "path": path,
            "map_from_frame": map_from_frame,
            "overlay_warnings": overlay_warnings,
        }

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        t = msg.twist.twist
        if not -0.1 <= age < (
            self.config.get("localization_timeout_s", 3.0) if self.hardware else 1
        ) or not all(math.isfinite(v) for v in (t.linear.x, t.linear.y, t.angular.z)):
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

    def motion_allowed(self, task):
        return True

    def controller_id(self):
        return "FollowPath"

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
            and self.motion_allowed(state)
            and time.monotonic() - self.raw_received
            < (self.config["command_timeout_s"] if self.hardware else 0.5)
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
        timeout = getattr(self, "config", {}).get("navigation_recovery_timeout_s", 5.0)
        deadline = time.monotonic() + timeout
        was_stopping = self.stopping
        while True:
            if self.interrupted(task_id):
                raise InterruptedError("Task canceled or superseded")
            status = self.status(False)
            position = self.pose()
            if (
                status["ready"]
                and position is not None
                and all(math.isfinite(position[k]) for k in ("x", "y", "yaw"))
                and 0
                <= position.get("age_s", math.inf)
                < getattr(self, "config", {}).get("pose_timeout_s", 3.0)
            ):
                self.stopping = was_stopping
                return position
            self.stopping = True
            self.command_pub.publish(Twist())
            health = status.get("health", {})
            if getattr(self, "clock_rewound", False) or any(
                health.get(key) is False
                for key in (
                    "exclusive_owners",
                    "motion_enabled",
                    "production_mode",
                    "service_active",
                )
            ):
                raise RuntimeError(
                    "Navigation disabled or ownership/clock conflict; stopped"
                )
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Navigation recovery timed out after %.1f s (%s); stopped"
                    % (timeout, ", ".join(status.get("not_ready_reasons", [])))
                )
            await self.pause(0.1)

    async def result(self, future, task_id, timeout=10):
        began = time.monotonic()
        while not future.done():
            if self.interrupted(task_id):
                raise InterruptedError("Task canceled or superseded")
            if not self.status(False)["ready"]:
                paused = time.monotonic()
                await self.wait_for_ready_pose(task_id)
                began += time.monotonic()-paused
            if future.done():
                break
            if time.monotonic() - began > timeout:
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
                time.monotonic() - self.velocity_at
                < (self.config.get("localization_timeout_s", 3.0) if self.hardware else 1)
                and self.velocity[0]
                <= (
                    self.config.get("stopped_linear_mps", 0.01)
                    if self.hardware
                    else 0.01
                )
                and self.velocity[1]
                <= (
                    self.config.get("stopped_angular_rps", 0.01)
                    if self.hardware
                    else 0.01
                )
            )
            stable = (now if stable is None else stable) if stopped else None
            if stable is not None and now - stable >= (
                self.config.get("stop_hold_s", 1.0) if self.hardware else 1.0
            ):
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
        """Join sensor-costmap NavFn legs into one continuous controller path.

        Explicit starts prevent every leg from being planned from the robot.
        Preserve the planner's obstacle detours; do not substitute road lines or
        write the reference geometry into either costmap.
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        start = validate_point(position)
        window = list(window)
        skipped = set()
        for offset, index in enumerate(window):
            point = guidance.points[index]
            if index != len(guidance.points) - 1:
                point = free_via_point(point, self.global_map)
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
            planner = await self.result(
                self.path_client.send_goal_async(request), task_id
            )
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
                now_s = self.get_clock().now().nanoseconds / 1e9
                if (index < len(guidance.points)-1
                        and self.scan is not None
                        and healthy_scan(self.scan, now_s, self.config['sensor_timeout_s'])
                        and observed_obstacle(guidance.points[index], self.global_map,
                                              position, now_s, scan=self.scan)):
                    next_index = index+1
                    next_point = guidance.points[next_index]
                    if (len(window) < 12
                            and math.hypot(next_point['x']-position['x'],
                                           next_point['y']-position['y']) <= 6.0):
                        if offset+1 == len(window):
                            window.append(next_index)
                        skipped.add(index)
                        continue
                # A sampled station may itself be occupied. Try the next station
                # on the SAME straight edge, allowing NavFn to go around it.
                # Never skip a bend or the window/final goal to obtain a shortcut.
                if index not in guidance.bends and offset + 1 < len(window):
                    next_point = guidance.points[window[offset + 1]]
                    if (
                        math.hypot(
                            next_point["x"] - start["x"], next_point["y"] - start["y"]
                        )
                        <= 4.1
                    ):
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
        # Commit skipped stations only after a complete obstacle-aware path
        # to a later station succeeds; a failed detour must not advance progress.
        guidance.skipped.update(skipped)
        for a, b in zip(path.poses, path.poses[1:]):
            heading = math.atan2(
                b.pose.position.y - a.pose.position.y,
                b.pose.position.x - a.pose.position.x,
            )
            a.pose.orientation.z, a.pose.orientation.w = (
                math.sin(heading / 2),
                math.cos(heading / 2),
            )
        return path

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
            state = self.tasks.snapshot(task_id, include_paths=True)
            if self.interrupted(task_id):
                raise InterruptedError("Task canceled before planning")
            self.active_follow_requests, self.active_follow_results = follow_requests, follow_results
            self.tasks.update(task_id, state="planning")
            position = await self.wait_for_ready_pose(task_id)
            # Preview and execution use exactly the same road-preference policy.
            route = {"reference_path": state["reference_path"], "goal": state["goal"]}
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
            budget = ProgressBudget(
                planner_patience_s=policy.get("planner_patience_s", 0.0),
                controller_patience_s=policy.get("controller_patience_s", 20.0),
            )
            started_sim = self.get_clock().now().nanoseconds / 1e9
            controller_used = None
            recovery_used = getattr(self, "handled_recovery", 0)
            while True:
                if self.interrupted(task_id):
                    raise InterruptedError("Task canceled or superseded")
                position = await self.wait_for_ready_pose(task_id)
                recovery = getattr(self, "handled_recovery", 0)
                if recovery != recovery_used:
                    current_points, follow_result, sent_window = [], None, None
                    guidance = RouteGuidance(route["reference_path"], position, goal_point)
                    budget = ProgressBudget(planner_patience_s=policy.get("planner_patience_s", 0.0),
                                            controller_patience_s=policy.get("controller_patience_s", 20.0))
                    recovery_used = recovery
                controller_id = self.controller_id()
                if controller_used is not None and controller_id != controller_used:
                    self.stopping = True
                    if not await self.release_controller(follow_requests, follow_results):
                        raise RuntimeError("Cannot release controller at gait boundary")
                    current_points, follow_result, sent_window = [], None, None
                controller_used = controller_id
                now = self.get_clock().now().nanoseconds / 1e9
                budget.moved(position["x"], position["y"], now)
                window = guidance.window(position)
                guidance_changed = window != sent_window
                remaining = math.dist(
                    (position["x"], position["y"]), (goal_point["x"], goal_point["y"])
                )
                route_remaining = guidance.remaining_length(position)
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
                if (
                    remaining <= tolerance
                    and route_remaining <= tolerance
                    and self.arrival_heading_ok(position, goal_point)
                ):
                    self.stopping = True
                    if not await self.release_controller(
                        follow_requests, follow_results
                    ):
                        raise RuntimeError("Could not release controller at arrival")
                    if not await self.confirm_stop():
                        raise RuntimeError("Could not confirm robot stop at arrival")
                    if self.hardware:
                        await self.wait_for_arrival_confirmation(task_id)
                    if self.interrupted(task_id):
                        raise InterruptedError("Task canceled while stopping")
                    if not self.status(False)["ready"]:
                        raise RuntimeError("Localization or sensors lost at arrival")
                    stopped_pose = await self.wait_for_ready_pose(task_id)
                    if self.hardware and getattr(self, "handled_recovery", 0) != recovery_used:
                        continue
                    remaining = math.dist(
                        (stopped_pose["x"], stopped_pose["y"]),
                        (goal_point["x"], goal_point["y"]),
                    )
                    self.tasks.update(task_id, distance_remaining_m=remaining)
                    if remaining <= tolerance and self.arrival_heading_ok(
                        stopped_pose, goal_point
                    ):
                        outcome, stop_confirmed = "succeeded", True
                        completion_pose = stopped_pose
                        break
                    # The estimate can move while braking. Resume planning rather
                    # than reporting arrival outside the unchanged tolerance.
                    current_points, follow_result = [], None
                    if self.hardware:
                        continue
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
                    if (
                        ended != GoalStatus.STATUS_SUCCEEDED
                        and not budget.controller_ended(now)
                    ):
                        raise RuntimeError(
                            "Controller could not make progress on this road"
                        )
                if not current_points or guidance_changed or now - last_plan >= 1.0:
                    last_plan = now
                    path = await self.guided_path(guidance, window, position, task_id)
                    points = (
                        [
                            {"x": p.pose.position.x, "y": p.pose.position.y}
                            for p in path.poses
                        ]
                        if path is not None
                        else []
                    )
                    if len(points) < 2:
                        if not budget.planned(False, now):
                            raise RuntimeError(
                                "No traversable path in the sensor costmap"
                            )
                        self.stopping = True
                        if not await self.release_controller(
                            follow_requests, follow_results
                        ):
                            raise RuntimeError(
                                "Cannot release controller while waiting for a path"
                            )
                        current_points, follow_result = [], None
                        self.tasks.update(
                            task_id,
                            state="planning",
                            reason="暂时无可行路径，停车等待并重规划",
                        )
                        await self.pause(0.5)
                        continue
                    budget.planned(True, now)
                    if (
                        not guidance_changed
                        and current_points
                        and follow_result
                        and not follow_result.done()
                    ):
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
                    command.controller_id = controller_id
                    command.goal_checker_id = "goal_checker"
                    if self.hardware and (self.geo.get("fusion") or {}).get("recovery_id", 0) != getattr(self, "handled_recovery", 0):
                        continue
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
                    self.active_controller = controller_id
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
                released = await self.release_controller(
                    follow_requests, follow_results
                )
                stopped = stop_confirmed or await self.confirm_stop()
                if not released or not stopped:
                    self.stop_failure = (
                        ((reason + "; ") if reason else "")
                        + "Could not confirm controller release and robot stop; waiting for verified stop"
                    )
                    self.stop_recovery = (follow_requests, follow_results)
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
        self.server.destroy()
        super().destroy_node()
