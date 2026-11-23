"""Run staged Ranger navigation using the executor's existing action clients."""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException

from .core.descent import Survey
from .core.ground_support import ACK_TOPIC, FRAME, SEGMENT_SPEED
from .ground_support_node import SupportFeed, latest_base, xy_yaw


class RangerDescent:
    def __init__(self, node):
        self.node = node
        self.feed = SupportFeed(node, node.config, node.tf)
        self.turn = 0.
        self.ack = None
        node.create_subscription(String, ACK_TOPIC, self.on_ack, 10)

    def on_ack(self, msg):
        source, _, stamp = msg.data.rpartition(":")
        if source.rsplit("/", 1)[-1] == "global_costmap":
            try:
                self.ack = int(stamp) / 1e9
            except ValueError:
                self.ack = None

    async def wait_support(self, task, timeout):
        node = self.node
        deadline = time.monotonic() + timeout
        while True:
            if node.interrupted(task):
                raise InterruptedError("Ranger task canceled while waiting for support")
            sample = self.feed.fresh()
            if sample is not None:
                try:
                    return sample, self.pose()
                except (TransformException, ValueError):
                    pass
            if time.monotonic() >= deadline:
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
        tf = self.node.tf.lookup_transform(target, source, Time())
        translation, yaw = xy_yaw(tf)
        x, y = xy
        return (translation[0]+math.cos(yaw)*x-math.sin(yaw)*y,
                translation[1]+math.sin(yaw)*x+math.cos(yaw)*y)

    def stamped(self, point, yaw, frame):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = map(float, point)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return pose

    async def path(self, task, xy, target, final_yaw):
        node = self.node
        sample = self.feed.fresh()
        if sample is None:
            return None
        requested_stamp = sample.stamp
        deadline = time.monotonic() + .6
        while self.ack is None or self.ack < requested_stamp:
            if time.monotonic() >= deadline:
                return None
            if node.interrupted(task):
                raise InterruptedError("Ranger planning canceled")
            if self.feed.fresh() is None:
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
            return None
        try:
            result = await node.result(handle.get_result_async(), task, timeout=3.)
        except BaseException:
            handle.cancel_goal_async()
            raise
        path = result.result.path
        if result.status != GoalStatus.STATUS_SUCCEEDED or len(path.poses) < 2:
            return None
        points = [self.transform(FRAME, path.header.frame_id,
                                 (pose.pose.position.x, pose.pose.position.y)) for pose in path.poses]
        sample = self.feed.fresh()
        if sample is None or math.dist(points[-1], target) > .2 or not sample.corridor(xy, points):
            return None
        return path, points

    async def follow(self, task, path, points, requests, results):
        node = self.node
        sample, (xy, yaw) = await self.wait_support(task, 1.25)
        ahead = next((point for point in points if math.dist(xy, point) >= .30), points[-1])
        heading = math.atan2(ahead[1]-xy[1], ahead[0]-xy[0])
        angle = math.remainder(heading-yaw, math.tau)
        if abs(angle) > .45:
            await self.observe(task, angle)
            sample, (xy, yaw) = await self.wait_support(task, 1.25)
            if abs(math.remainder(heading-yaw, math.tau)) > .55:
                return False
        if not sample.corridor(xy, points):
            return False
        command = FollowPath.Goal()
        command.path, command.controller_id, command.goal_checker_id = path, "FollowPath", "goal_checker"
        node.raw_command, node.raw_received = Twist(), 0.
        pending = node.follow_client.send_goal_async(command)
        requests.append(pending)
        follower = await node.result(pending, task, timeout=3.)
        if not follower.accepted:
            return False
        result = follower.get_result_async()
        results[pending] = result
        revision = node.tasks.snapshot(task, include_paths=False)["path_revision"] + 1
        node.tasks.update(task, state="navigating", reason="Ranger confirmed segment", path_revision=revision,
                          execution_path=[{"x": p.pose.position.x, "y": p.pose.position.y}
                                          for p in path.poses])
        node.path_pub.publish(path)
        node.stopping = False
        deadline = time.monotonic() + 32.
        try:
            while not result.done() and time.monotonic() < deadline:
                if node.interrupted(task):
                    raise InterruptedError("Ranger segment canceled")
                sample = self.feed.fresh()
                if sample is None:
                    break
                try:
                    xy, _ = self.pose()
                except (TransformException, ValueError):
                    break
                index = min(range(len(points)), key=lambda i: math.dist(xy, points[i]))
                if node.stopping or not sample.corridor(xy, points[index:]):
                    break
                await node.pause(.08)
        finally:
            node.stopping = True
            node.command_pub.publish(Twist())
            if not await node.release_controller(requests, results) or not await node.confirm_stop():
                raise RuntimeError("Could not confirm stop at Ranger segment boundary")
        status = result.result().status
        requests.clear()
        results.clear()
        return status == GoalStatus.STATUS_SUCCEEDED

    async def run(self, task, goal, guidance, tolerance, requests, results):
        node = self.node
        survey = Survey()
        deadline = time.monotonic() + 225.
        await self.wait_support(task, node.config["navigation_recovery_timeout_s"])
        needs_observation = False
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
            _, (xy, _) = await self.wait_support(task, 1.25)
            final_index = len(guidance.points) - 1
            bend = min((i for i in guidance.bends if i >= guidance.index), default=final_index)
            destination = guidance.target(bend)
            objective = self.transform(FRAME, "map", (destination["x"], destination["y"]))
            survey.moved(xy)
            if needs_observation:
                await self.observe(task, math.radians(25))
                needs_observation = False
            support, (xy, _) = await self.wait_support(task, 1.25)
            candidates = survey.candidates(support, xy, objective)
            selected = None
            for target in candidates:
                planned = await self.path(task, xy, target,
                                          goal.get("yaw") if target == objective and bend == final_index else None)
                if planned:
                    selected = target, planned
                    break
                survey.failed(target, support.resolution)
            if selected is None:
                consecutive_failures += 1
                await self.observe(task, math.pi / 2)
                continue
            target, (path, points) = selected
            finished = await self.follow(task, path, points, requests, results)
            _, (current, _) = await self.wait_support(task, 1.25)
            if finished or math.dist(xy, current) > .32:
                consecutive_failures = 0
                survey.visited.append(xy)
                needs_observation = True
            else:
                survey.failed(target, support.resolution)
                consecutive_failures += 1
                needs_observation = True
        raise RuntimeError("Ranger found no confirmed safe route within the search budget; stopped")
