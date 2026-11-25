"""Run staged Ranger navigation using the executor's existing action clients."""

import math
import time

import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import TransformException

from .core.descent import Survey
from .core.ground_support import ACK_TOPIC, DESCENT_ACTIVE_TOPIC, FRAME, SURVEY_ACTIVE_TOPIC
from .ground_support_node import SupportFeed, latest_base, xy_yaw
from .terrain_execution import rotation_clear


class RangerDescent:
    def __init__(self, node):
        self.node = node
        self.auto = node.config.get("ranger_descent_mode") == "auto"
        self.normal_feed = SupportFeed(node, node.config, node.tf, with_heights=True)
        self.survey_feed = (SupportFeed(node, node.config, node.tf, with_heights=True, survey=True)
                            if self.auto else None)
        self.turn = 0.
        self.ack = None
        self.takeover_wait = None
        self.active = False
        self.searching = False
        self.observation_wait = None
        self.observation_task = None
        self.observation_origin = None
        self.observation_attempts = 0
        if self.auto:
            self.active_pub = node.create_publisher(Bool, DESCENT_ACTIVE_TOPIC, 1)
            self.survey_pub = node.create_publisher(Bool, SURVEY_ACTIVE_TOPIC, 1)
        else:
            node.create_subscription(String, ACK_TOPIC, self.on_ack, 10)

    def publish_active(self):
        if self.auto:
            self.active_pub.publish(Bool(data=self.active))
            self.survey_pub.publish(Bool(data=self.searching))

    @property
    def feed(self):
        return self.survey_feed if self.searching else self.normal_feed

    def invalidate_support(self):
        self.normal_feed.invalidate()
        if self.survey_feed is not None:
            self.survey_feed.invalidate()

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

    def should_take_over(self, task):
        """A fresh ground hazard on the active route can start a stopped survey."""
        node = self.node
        now = time.monotonic()
        blockers = set(node.guard.get("blocked_reasons", ()))
        eligible = {"ranger_crest", "ground_hazard", "ground_drop_discontinuous"}
        ahead = node.guard.get("ranger_terrain_ahead")
        if (now - node.guard_at >= node.config["command_timeout_s"]
                or (not blockers and ahead not in eligible) or not blockers <= eligible
                or self.feed.fresh() is None):
            self.takeover_wait = None
            return False
        if self.takeover_wait is None or self.takeover_wait[0] != task:
            self.takeover_wait = (task, now)
        return now - self.takeover_wait[1] >= .35

    async def search_and_descend(self, task, goal, guidance, tolerance, requests, results):
        node = self.node
        blockers = node.guard.get("blocked_reasons", ())
        survey_edge = ("ground_hazard" in blockers
                       or node.guard.get("ranger_terrain_ahead") == "ground_hazard")
        self.takeover_wait = None
        node.stopping = True
        node.command_pub.publish(Twist())
        if not await node.release_controller(requests, results) or not await node.confirm_stop():
            raise RuntimeError("Cannot confirm stopped controller before Ranger descent")
        requests.clear()
        results.clear()
        await node.prepare_ranger_segment(task)
        self.searching = True
        try:
            self.publish_active()
            self.report("survey_started", self.pose()[0])
            await self.run(task, goal, guidance, tolerance, requests, results, survey_edge=survey_edge)
        finally:
            self.active = False
            self.searching = False
            self.publish_active()

    def landing_ready(self, sample, heights, xy, level, platform_z, heading):
        if level > platform_z - .10:
            return False
        radius = self.node.config["footprint_radius_m"]
        direction = np.array([math.cos(heading), math.sin(heading)])
        flat = abs(heights - level) <= .035
        return sample.corridor(np.asarray(xy) - radius * direction,
                               [np.asarray(xy) + (radius + .35) * direction], flat)

    async def verify_landing(self, task, first, platform_z, heading):
        deadline = time.monotonic() + self.node.config["ranger_depth_timeout_s"]
        while time.monotonic() < deadline:
            sample, heights, xy, level = await self.wait_support(task, 1.25, need_heights=True)
            if sample.stamp > first:
                if not self.landing_ready(sample, heights, xy, level, platform_z, heading):
                    return False
                normal = self.normal_feed.fresh()
                normal_heights = self.normal_feed.elevation(normal) if normal is not None else None
                ahead = (xy[0] + .60 * math.cos(heading), xy[1] + .60 * math.sin(heading))
                if (normal_heights is not None and normal.corridor(xy, [ahead])
                        and abs(normal.level(ahead, normal_heights) - level) <= .035):
                    return True
            await self.node.pause(.05)
        return False

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
        waiting_for = "no fresh ground grid"
        while True:
            if node.interrupted(task):
                raise InterruptedError("Ranger task canceled while waiting for support")
            sample = self.feed.fresh()
            if sample is not None:
                waiting_for = "no fresh pose"
                try:
                    pose = self.pose()
                    if not need_heights:
                        return sample, pose
                    heights = self.feed.elevation(sample)
                    xy, _ = pose
                    waiting_for = "no matching height grid" if heights is None else "current footprint unconfirmed"
                    if heights is not None and sample.clear([xy]):
                        x, y = ((point-origin)//sample.resolution
                                for point, origin in zip(xy, sample.origin))
                        height = float(heights[int(y), int(x)])
                        if math.isfinite(height):
                            return sample, heights, xy, height
                except (TransformException, ValueError):
                    pass
            if time.monotonic() >= deadline:
                self.report("ground_support_insufficient", None, detail=waiting_for)
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
                sample = self.feed.fresh()
                if sample is None:
                    break
                if self.auto and not rotation_clear(node, ground=sample if self.searching else None):
                    self.report("observation_turn_blocked", None, detail="body rotation envelope is not clear")
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
        self.active = self.auto
        self.publish_active()
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

    async def run(self, task, goal, guidance, tolerance, requests, results, *, survey_edge=False):
        node = self.node
        survey = Survey()
        deadline = time.monotonic() + 225.
        _, _, _, platform_z = await self.wait_support(
            task, node.config["navigation_recovery_timeout_s"], need_heights=True
        )
        scan_angle = math.radians(-60.) if survey_edge else None
        descent_heading = None
        consecutive_failures = 0
        support_timeout = node.config["navigation_recovery_timeout_s"] if self.auto else 1.25
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
            if self.auto and descent_heading is not None:
                sample, heights, xy, level = await self.wait_support(task, support_timeout, need_heights=True)
                if (self.landing_ready(sample, heights, xy, level, platform_z, descent_heading)
                        and await self.verify_landing(task, sample.stamp, platform_z, descent_heading)):
                    self.report("landing_confirmed_returning_to_nav2", xy)
                    return position
            node.tasks.update(task, state="planning", reason="Ranger observing confirmed ground")
            if scan_angle is not None:
                await self.observe(task, scan_angle)
                scan_angle = None
            support, heights, xy, ground_z = await self.wait_support(task, support_timeout, need_heights=True)
            final_index = len(guidance.points) - 1
            bend = min((i for i in guidance.bends if i >= guidance.index), default=final_index)
            destination = guidance.target(bend)
            objective = self.transform(FRAME, "map", (destination["x"], destination["y"]))
            survey.moved(xy)
            candidates = survey.staged_candidates(support, heights, xy, objective, platform_z, ground_z)
            if self.auto and descent_heading is not None:
                # Give the landing handoff a chance before taking the final goal in survey mode.
                candidates.sort(key=lambda item: item[0] == "goal")
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
            try:
                finished = await self.follow(task, path, points, requests, results, phase, platform_z)
            finally:
                self.active = False
                self.publish_active()
            if finished and phase in ("descent", "goal"):
                dest = points[-1]
                approach = next((p for p in reversed(points) if math.dist(p, dest) >= .4), points[0])
                descent_heading = math.atan2(dest[1] - approach[1], dest[0] - approach[0])
            _, (current, _) = await self.wait_support(task, support_timeout)
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
