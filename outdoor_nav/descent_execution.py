"""Run staged Ranger navigation using the executor's existing action clients."""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import TransformException

from .core.descent import Survey
from .core.ground_support import ACK_TOPIC, DESCENT_ACTIVE_TOPIC, FRAME
from .ground_support_node import SupportFeed, latest_base, xy_yaw
from .terrain_execution import rotation_clear


class RangerDescent:
    def __init__(self, node):
        self.node = node
        self.auto = node.config.get("ranger_descent_mode") == "auto"
        self.feed = SupportFeed(node, node.config, node.tf, with_heights=True)
        self.turn = 0.
        self.ack = None
        self.candidate = None
        self.active = False
        self.observation_wait = None
        self.observation_task = None
        self.observation_origin = None
        self.observation_attempts = 0
        if self.auto:
            self.active_pub = node.create_publisher(Bool, DESCENT_ACTIVE_TOPIC, 1)
        else:
            node.create_subscription(String, ACK_TOPIC, self.on_ack, 10)

    def publish_active(self):
        if self.auto:
            self.active_pub.publish(Bool(data=self.active))

    async def observe_unknown(self, task, requests, results):
        node = self.node
        if self.observation_task != task:
            self.observation_task = task
            self.observation_wait = self.observation_origin = None
            self.observation_attempts = 0
        blockers = node.guard.get("blocked_reasons", ())
        now = time.monotonic()
        if (now - node.guard_at >= node.config["command_timeout_s"]
                or set(blockers) != {"ground_unobserved"} or abs(node.velocity[0]) > .01):
            self.observation_wait = None
            return False
        if self.observation_wait is None:
            self.observation_wait = now
        if now - self.observation_wait < .75:
            return False
        sample = self.feed.fresh()
        try:
            xy, _ = self.pose()
        except (TransformException, ValueError):
            return False
        if sample is None or not sample.clear([xy]) or not rotation_clear(node):
            return False
        if self.observation_origin is None or math.dist(xy, self.observation_origin) > .25:
            self.observation_origin, self.observation_attempts = xy, 0
        if self.observation_attempts >= 2:
            return False
        angle = math.radians(35 if self.observation_attempts == 0 else -35)
        self.observation_attempts += 1
        self.observation_wait = None
        node.stopping = True
        node.command_pub.publish(Twist())
        if not await node.release_controller(requests, results) or not await node.confirm_stop():
            raise RuntimeError("Cannot confirm stop for Ranger observation")
        requests.clear()
        results.clear()
        node.tasks.update(task, state="planning", reason="Ranger checking unobserved ground")
        await self.observe(task, angle)
        if rotation_clear(node):
            await self.observe(task, -angle)
        return True

    def downhill_candidate(self, position, points):
        if len(points) < 2:
            self.candidate = None
            return None
        sample = self.feed.fresh()
        if sample is None:
            self.candidate = None
            return None
        heights = self.feed.elevation(sample)
        if heights is None:
            return None
        try:
            xy, _ = self.pose()
            pose = position["x"], position["y"]
            route = [(p["x"], p["y"]) for p in points]
            index = min(range(len(route)), key=lambda i: math.dist(pose, route[i]))
            profile = sample.profile(xy, self.transform_points(FRAME, "map", route[index+1:]), heights)
        except (TransformException, ValueError):
            self.candidate = None
            return None
        if profile.crest_m is None or profile.crest_m > .65 or profile.landing is None:
            self.candidate = None
            return None
        previous = self.candidate
        self.candidate = sample.stamp, profile.crest
        if (previous is not None and 0 < sample.stamp - previous[0] <= self.node.config["ranger_depth_timeout_s"]
                and math.dist(profile.crest, previous[1]) <= .25):
            return profile.landing
        return None

    async def descend(self, task, target, requests, results):
        node = self.node
        self.candidate = None
        node.stopping = True
        node.command_pub.publish(Twist())
        if not await node.release_controller(requests, results) or not await node.confirm_stop():
            raise RuntimeError("Cannot confirm stopped controller before Ranger descent")
        requests.clear()
        results.clear()
        _, (xy, _) = await self.wait_support(task, 1.25)
        planned = await self.path(task, xy, target, None, "descent", None)
        if planned is None:
            raise RuntimeError("Ranger descent has no fully supported path; stopped")
        path, points = planned
        await node.prepare_ranger_segment(task)
        try:
            if not await self.follow(task, path, points, requests, results, "descent", None):
                raise RuntimeError("Ranger descent segment did not complete; stopped")
        finally:
            self.active = False
            self.publish_active()
        await self.confirm_landing(task, target, points)

    async def confirm_landing(self, task, target, points):
        node = self.node
        anchor = next((point for point in reversed(points) if math.dist(point, target) >= .4), points[0])
        heading = math.atan2(target[1] - anchor[1], target[0] - anchor[0])
        deadline = time.monotonic() + node.config["navigation_recovery_timeout_s"]
        previous = None
        while time.monotonic() < deadline:
            sample, (_, heights, xy, level) = await self.wait_support(task, 1.25, need_heights=True)
            probe = xy[0] + .45 * math.cos(heading), xy[1] + .45 * math.sin(heading)
            if (math.dist(xy, target) <= .30 and sample.corridor(xy, [probe])
                    and abs(sample.level(probe, heights) - level) <= .035):
                if previous is not None and 0 < sample.stamp - previous <= node.config["ranger_depth_timeout_s"]:
                    return
                previous = sample.stamp
            else:
                previous = None
            await node.pause(.05)
        raise RuntimeError("Ranger flat landing is not confirmed; stopped")

    def on_ack(self, msg):
        source, _, stamp = msg.data.rpartition(":")
        if source.rsplit("/", 1)[-1] == "global_costmap":
            try:
                self.ack = int(stamp) / 1e9
            except ValueError:
                self.ack = None

    def report(self, reason, xy, target=None, detail=""):
        self.node.get_logger().info(f"Ranger {reason} pose={xy} target={target} {detail}")

    async def wait_support(self, task, timeout, *, need_heights=False):
        node = self.node
        deadline = time.monotonic() + timeout
        while True:
            if node.interrupted(task):
                raise InterruptedError("Ranger task canceled while waiting for support")
            sample = self.feed.fresh()
            if sample is not None:
                try:
                    pose = self.pose()
                    if not need_heights:
                        return sample, pose
                    heights = self.feed.elevation(sample)
                    xy, _ = pose
                    if heights is not None and sample.clear([xy]):
                        x, y = ((point-origin)//sample.resolution
                                for point, origin in zip(xy, sample.origin))
                        height = float(heights[int(y), int(x)])
                        if math.isfinite(height):
                            return sample, heights, xy, height
                except (TransformException, ValueError):
                    pass
            if time.monotonic() >= deadline:
                self.report("ground_support_insufficient", None, detail="fresh observation or pose unavailable")
                raise RuntimeError("Ranger depth or support data lost; stopped")
            await node.pause(.05)

    def pose(self):
        return xy_yaw(latest_base(self.node.tf, self.node, self.node.config["ranger_depth_timeout_s"]))

    async def observe(self, task, angle):
        node = self.node
        _, (_, previous) = await self.wait_support(task, 1.25)
        direction = math.copysign(1., angle)
        self.turn = direction * min(.35, node.config["max_angular_speed_rps"])
        swept = 0.
        deadline = time.monotonic() + abs(angle/self.turn) + 6.
        try:
            while swept < abs(angle) and time.monotonic() < deadline:
                if node.interrupted(task):
                    raise InterruptedError("Ranger observation canceled")
                if self.auto and not rotation_clear(node):
                    break
                sample = self.feed.fresh()
                if sample is None:
                    break
                try:
                    xy, yaw = self.pose()
                except (TransformException, ValueError):
                    break
                if not sample.clear([xy]):
                    break
                swept += direction * math.remainder(yaw-previous, math.tau)
                previous = yaw
                await node.pause(.05)
        finally:
            self.turn = 0.
            node.command_pub.publish(Twist())
        if not await node.confirm_stop():
            raise RuntimeError("Could not confirm stop after Ranger observation")

    def transform(self, target, source, xy):
        return self.transform_points(target, source, [xy])[0]

    def transform_points(self, target, source, points):
        tf = self.node.tf.lookup_transform(target, source, Time())
        translation, yaw = xy_yaw(tf)
        c, s = math.cos(yaw), math.sin(yaw)
        return [(translation[0]+c*x-s*y, translation[1]+s*x+c*y) for x, y in points]

    def stamped(self, point, yaw, frame):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = map(float, point)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return pose

    async def path(self, task, xy, target, final_yaw, phase, platform_z):
        node = self.node
        sample = self.feed.fresh()
        if sample is None or not sample.clear([xy, target]):
            self.report("ground_support_insufficient", xy, target, "start or target unsupported")
            return None
        requested_stamp = sample.stamp
        deadline = time.monotonic() + .6
        while not self.auto and (self.ack is None or self.ack < requested_stamp):
            if time.monotonic() >= deadline:
                self.report("navfn_no_path", xy, target, "costmap did not acknowledge current support")
                return None
            if node.interrupted(task):
                raise InterruptedError("Ranger planning canceled")
            if self.feed.fresh() is None:
                self.report("ground_support_insufficient", xy, target, "support expired awaiting costmap")
                return None
            await node.pause(.05)
        request = ComputePathToPose.Goal()
        start_map = self.transform("map", FRAME, xy)
        target_map = self.transform("map", FRAME, target)
        heading = math.atan2(target_map[1]-start_map[1], target_map[0]-start_map[0])
        request.start = self.stamped(start_map, heading, "map")
        request.goal = self.stamped(target_map, heading if final_yaw is None else final_yaw, "map")
        request.use_start = True
        request.planner_id = "GridBased"
        handle = await node.result(node.path_client.send_goal_async(request), task, timeout=3.)
        if not handle.accepted:
            self.report("navfn_no_path", xy, target, "planner rejected the request")
            return None
        try:
            result = await node.result(handle.get_result_async(), task, timeout=3.)
        except (TimeoutError, RuntimeError) as exc:
            handle.cancel_goal_async()
            self.report("navfn_no_path", xy, target, str(exc))
            return None
        except BaseException:
            handle.cancel_goal_async()
            raise
        path = result.result.path
        if result.status != GoalStatus.STATUS_SUCCEEDED or len(path.poses) < 2:
            self.report("navfn_no_path", xy, target, f"status={result.status} poses={len(path.poses)}")
            return None
        points = self.transform_points(FRAME, path.header.frame_id,
                                       [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses])
        if math.dist(points[-1], target) > .2:
            self.report("path_recheck_failed", xy, target, "NavFn endpoint missed the candidate")
            return None
        if not self.route_supported(self.feed.fresh(), xy, points, phase, platform_z):
            self.report("path_recheck_failed", xy, target, "returned path lacks current support or leaves the platform")
            return None
        return path, points

    def route_supported(self, sample, xy, points, phase, platform_z):
        if sample is None:
            return False
        allowed = None
        if phase == "platform":
            heights = self.feed.elevation(sample)
            if heights is None:
                return False
            allowed = Survey.platform(heights, platform_z)
        return sample.corridor(xy, points, allowed)

    async def follow(self, task, path, points, requests, results, phase, platform_z):
        node = self.node
        sample, (xy, yaw) = await self.wait_support(task, 1.25)
        ahead = next((point for point in points if math.dist(xy, point) >= .30), points[-1])
        heading = math.atan2(ahead[1]-xy[1], ahead[0]-xy[0])
        angle = math.remainder(heading-yaw, math.tau)
        if abs(angle) > .45:
            await self.observe(task, angle)
            sample, (xy, yaw) = await self.wait_support(task, 1.25)
            if abs(math.remainder(heading-yaw, math.tau)) > .55:
                self.report("tracking_aborted", xy, points[-1], "could not align at a stop")
                return False
        if not self.route_supported(sample, xy, points, phase, platform_z):
            self.report("path_recheck_failed", xy, points[-1], "path changed before tracking")
            return False
        if self.auto:
            profile = sample.profile(xy, points, self.feed.elevation(sample), landing_distance=.45)
            if profile.landing is None:
                self.report("path_recheck_failed", xy, points[-1], "continuous descent and landing not confirmed")
                return False
            self.active = True
        command = FollowPath.Goal()
        command.path = path
        command.controller_id = "RangerFollowPath" if self.auto else "FollowPath"
        command.goal_checker_id = "ranger_goal_checker" if self.auto else "goal_checker"
        node.raw_command, node.raw_received = Twist(), 0.
        pending = node.follow_client.send_goal_async(command)
        requests.append(pending)
        follower = await node.result(pending, task, timeout=3.)
        if not follower.accepted:
            self.report("tracking_aborted", xy, points[-1], "controller rejected the path")
            return False
        result = follower.get_result_async()
        results[pending] = result
        revision = node.tasks.snapshot(task, include_paths=False)["path_revision"] + 1
        node.tasks.update(task, state="navigating", reason=f"Ranger {phase} segment", path_revision=revision,
                          execution_path=[{"x": p.pose.position.x, "y": p.pose.position.y}
                                          for p in path.poses])
        node.path_pub.publish(path)
        node.stopping = False
        deadline = time.monotonic() + 32.
        abort_reason = None
        try:
            while not result.done() and time.monotonic() < deadline:
                if node.interrupted(task):
                    raise InterruptedError("Ranger segment canceled")
                sample = self.feed.fresh()
                if sample is None:
                    abort_reason = "ground_support_insufficient: support expired in motion"
                    break
                try:
                    xy, _ = self.pose()
                except (TransformException, ValueError):
                    abort_reason = "ground_support_insufficient: pose unavailable in motion"
                    break
                index = min(range(len(points)), key=lambda i: math.dist(xy, points[i]))
                if node.stopping:
                    abort_reason = "execution requested a stop"
                    break
                if phase == "platform" and self.feed.elevation(sample) is None:
                    # DDS can deliver the two messages in either order. The
                    # mask still protects motion until the matching heights arrive.
                    if sample.corridor(xy, points[index:]):
                        await node.pause(.05)
                        continue
                if not self.route_supported(sample, xy, points[index:], phase, platform_z):
                    abort_reason = "path_recheck_failed: remaining corridor lost support"
                    break
                await node.pause(.08)
        finally:
            completed = result.done()
            node.stopping = True
            node.command_pub.publish(Twist())
            if not await node.release_controller(requests, results) or not await node.confirm_stop():
                raise RuntimeError("Could not confirm stop at Ranger segment boundary")
        status = result.result().status
        requests.clear()
        results.clear()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.report("segment_completed", xy, points[-1], phase)
            return True
        self.report("tracking_aborted", xy, points[-1], f"{phase}: {abort_reason or ('controller ended the segment' if completed else 'segment time limit')}, status={status}")
        return False

    async def run(self, task, goal, guidance, tolerance, requests, results):
        node = self.node
        survey = Survey()
        deadline = time.monotonic() + 225.
        _, _, _, platform_z = await self.wait_support(
            task, node.config["navigation_recovery_timeout_s"], need_heights=True
        )
        scan_angle = None
        consecutive_failures = 0
        while time.monotonic() < deadline and len(survey.visited) < 18 and consecutive_failures < 8:
            position = await node.wait_for_ready_pose(task)
            guidance.window(position)
            node.tasks.update(task, distance_remaining_m=guidance.remaining_length(position))
            if (guidance.final_leg
                    and math.hypot(position["x"]-goal["x"], position["y"]-goal["y"]) <= min(.25, tolerance)
                    and node.arrival_heading_ok(position, goal)):
                await self.wait_support(task, 1.25)
                await node.wait_for_arrival_confirmation(task)
                position = await node.wait_for_ready_pose(task)
                await self.wait_support(task, 1.25)
                if (math.hypot(position["x"]-goal["x"], position["y"]-goal["y"]) <= min(.25, tolerance)
                        and node.arrival_heading_ok(position, goal)):
                    return position
            node.tasks.update(task, state="planning", reason="Ranger observing confirmed ground")
            if scan_angle is not None:
                await self.observe(task, scan_angle)
                scan_angle = None
            support, heights, xy, ground_z = await self.wait_support(task, 1.25, need_heights=True)
            final_index = len(guidance.points) - 1
            bend = min((i for i in guidance.bends if i >= guidance.index), default=final_index)
            destination = guidance.target(bend)
            objective = self.transform(FRAME, "map", (destination["x"], destination["y"]))
            survey.moved(xy)
            candidates = survey.staged_candidates(support, heights, xy, objective, platform_z, ground_z)
            if not candidates:
                self.report("ground_support_insufficient", xy, detail=f"no eligible staged target ground_z={ground_z:.3f}")
            selected = None
            for phase, target in candidates:
                planned = await self.path(task, xy, target,
                                          goal.get("yaw") if target == objective and bend == final_index else None,
                                          phase, platform_z)
                if planned:
                    selected = phase, target, planned
                    break
                survey.failed(target, support.resolution)
            if selected is None:
                consecutive_failures += 1
                angle = 90 if abs(ground_z - platform_z) <= .06 else 25
                scan_angle = math.radians(angle)
                continue
            phase, target, (path, points) = selected
            finished = await self.follow(task, path, points, requests, results, phase, platform_z)
            _, (current, _) = await self.wait_support(task, 1.25)
            if finished or math.dist(xy, current) > .32:
                consecutive_failures = 0
                if phase == "platform":
                    survey.visited.append(current)
                scan_angle = math.radians(90 if phase == "platform" else 25)
            else:
                survey.failed(target, support.resolution)
                consecutive_failures += 1
                scan_angle = math.radians(25)
        self.report("search_exhausted", self.pose()[0], detail=f"consecutive_failures={consecutive_failures}")
        raise RuntimeError("Ranger found no confirmed safe route within the search budget; stopped")
