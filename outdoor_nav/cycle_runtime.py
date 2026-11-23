"""Acquire, freeze, arbitrate and plan once per absolute decision deadline."""
import json
import math
import time
from array import array
from collections import deque
from threading import Lock
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import TransformException
from .core.heuristic import HintQueue, average_hint, body_delta
from .core.cycle_grid import fuse_grid
from .core.tasks import ProgressBudget
from .core.scan_health import acquisition_time, healthy_scan


class SnapshotUnavailable(ValueError):
    """A sensor or its acquisition-time transform has not arrived yet."""


class CycleNavigation:
    def __init__(self, node):
        self.n, self.p = node, node.runtime['decision_cycle']
        self.input_lock = Lock()
        self.samples = deque(maxlen=self.p['sample_limit'])
        self.scan = self.camera = None
        self.camera_stamp = -math.inf
        self.scans = deque(maxlen=64)
        self.global_ack = None
        self.number, self.valid_until = 0, -math.inf
        self.active_result = None
        self.report = {'enabled': True, 'ready': False}
        self.history = deque(maxlen=100)
        t = node.runtime['topic_remap']
        self.pub = node.create_publisher(OccupancyGrid, t['cycle_costmap'], 1)
        node.create_subscription(String, t['cycle_ack'], self.ack, 10)
        if node.config['lidar_enabled']:
            node.create_subscription(LaserScan, t['scan'], self.on_scan, qos_profile_sensor_data)
        if node.config['camera_enabled']:
            node.create_subscription(OccupancyGrid, t['camera_costmap'], self.on_camera, 5)

    def on_scan(self, msg):
        with self.input_lock:
            if self.scan is None or acquisition_time(msg) > acquisition_time(self.scan):
                self.scan = msg
                self.scans.append(msg)

    def on_camera(self, msg):
        # Invalid certificates produce a fresh, entirely unknown grid.
        # Its arrival must invalidate the handoff, not renew sensor freshness.
        valid = (msg.header.frame_id == 'odom'
                 and len(msg.data) == msg.info.width * msg.info.height
                 and np.any(np.frombuffer(msg.data, dtype=np.int8) >= 0))
        stamp = acquisition_time(msg)
        with self.input_lock:
            if stamp >= self.camera_stamp:
                self.camera_stamp = stamp
                self.camera = msg if valid else None

    def ack(self, msg):
        try:
            node, stamp = msg.data.rsplit(':', 1)
            if node.rstrip('/').split('/')[-1] == 'global_costmap':
                self.global_ack = int(stamp)
        except (ValueError, TypeError):
            pass

    def sample(self, status):
        sample = status.get('navigation_sample')
        if not status.get('ready') or not isinstance(sample, dict):
            return
        try:
            if (sample['schema_version'] != 1 or sample['reference_point'] != 'base_link'
                    or sample['local_frame'] != 'odom' or sample['global_frame'] != 'map'
                    or not isinstance(sample['epoch'], str)
                    or not all(math.isfinite(sample[p][k]) for p in ('map_pose','odom_pose') for k in ('x','y','yaw'))):
                return
            stamp = sample['stamp_s']
            if not math.isfinite(stamp) or stamp <= 0:
                return
            with self.input_lock:
                if self.samples and sample['epoch'] != self.samples[-1]['epoch']:
                    self.valid_until = -math.inf
                    self.samples.clear()
                    self.scan = self.camera = None
                    self.camera_stamp = -math.inf
                    self.scans.clear()
                if (not self.samples or (stamp > self.samples[-1]['stamp_s'] and
                        sample.get('sample_id',stamp) != self.samples[-1].get('sample_id',self.samples[-1]['stamp_s']))):
                    self.samples.append(sample)
        except (TypeError, KeyError):
            return

    def allowed(self):
        if time.monotonic() >= self.valid_until:
            return False
        if self.active_result is not None and self.active_result.done():
            return False
        now = self.n.get_clock().now().nanoseconds / 1e9
        age = self.n.config['sensor_timeout_s']
        with self.input_lock:
            scan, camera = self.scan, self.camera
        return (
            (not self.n.config['lidar_enabled'] or
             scan is not None and 0 <= now-acquisition_time(scan) and healthy_scan(scan, now, age))
            and (not self.n.config['camera_enabled'] or
                 camera is not None and 0 <= now-acquisition_time(camera) < age)
        )

    def paired_samples(self, now):
        age = min(self.p['period_s'], self.n.config['localization_timeout_s'])
        with self.input_lock:
            return self._fresh_samples(now, age)

    def _fresh_samples(self, now, age):
        return [s for s in self.samples if 0 <= now-s['stamp_s'] < age]

    def snapshot_inputs(self):
        age = min(self.p['period_s'], self.n.config['localization_timeout_s'])
        with self.input_lock:
            now = self.n.get_clock().now().nanoseconds / 1e9
            return now, self._fresh_samples(now, age), (tuple(self.scans), self.camera)

    async def future(self, future, task, deadline):
        while True:
            if self.n.interrupted(task):
                raise InterruptedError('Task canceled during cycle')
            if time.monotonic() >= deadline:
                raise TimeoutError('Decision cycle deadline exceeded')
            if future.done():
                return future.result()
            await self.n.pause(.01)

    async def execute(self, handle):
        n = self.n
        task = bytes(handle.goal_id.uuid).hex()
        requests, results = [], {}
        outcome, reason = 'failed', None
        try:
            while n.owner is not None:
                if n.interrupted(task):
                    raise InterruptedError('Canceled before execution')
                await n.pause()
            with n.state_lock:
                n.owner, n.stopping = task, True
                n.active_follow_requests, n.active_follow_results = requests, results
            item = n.tasks.snapshot(task)
            n.tasks.update(task, state='planning')
            await n.wait_for_ready_pose(task)
            await n.prepare_route(item, task)
            pool = item['reference_path']
            queue = HintQueue(pool, self.p['hint_spacing_m'], self.p['corner_error_m'], self.p['arrival_side_m'])
            budget = ProgressBudget(planner_patience_s=n.config['planner_patience_s'],
                                    controller_patience_s=n.config['controller_patience_s'])
            controller_attempted = False
            boundary = time.monotonic()
            task_epoch = None
            terrain_stamp = -math.inf
            while not n.interrupted(task):
                delay = boundary-time.monotonic()
                if delay > 0:
                    await n.pause(delay)
                started = time.monotonic()
                # Missed slots are skipped, never replayed as a burst of decisions.
                boundary += (max(0, math.floor((started-boundary)/self.p['period_s']))+1)*self.p['period_s']
                self.number += 1
                timings = {}
                handoff_started = False
                try:
                    requests[:] = [r for r in requests if r not in results or not results[r].done()]
                    results = {r:f for r,f in results.items() if r in requests}
                    n.active_follow_requests, n.active_follow_results = requests, results
                    if await n.synchronize_localization(task):
                        requests.clear()
                        results.clear()
                        self.active_result, self.valid_until = None, -math.inf
                    state_started=time.monotonic()
                    ros_now, samples, observations = self.snapshot_inputs()
                    if any(not r.done() for r in requests):
                        raise ValueError('Previous controller acceptance is still pending')
                    status = n.status(False)
                    if not status['ready']:
                        raise ValueError('Cycle inputs not ready: '+', '.join(status.get('not_ready_reasons', ())))
                    if not samples:
                        raise SnapshotUnavailable('No fresh paired localization sample')
                    if task_epoch is not None and samples[-1]['epoch'] != task_epoch:
                        raise RuntimeError('Localization epoch changed; restart navigation task')
                    task_epoch = samples[-1]['epoch']
                    position = samples[-1]['odom_pose']
                    budget.moved(position['x'],position['y'],time.monotonic())
                    hint = average_hint(queue.target, samples)
                    if queue.advance(hint, position):
                        hint = average_hint(queue.target, samples)
                    x,y = body_delta(hint, position)
                    final_tolerance = item['arrival_tolerance_m']
                    if queue.final and math.hypot(x,y) <= final_tolerance:
                        actual = body_delta(queue.target, samples[-1]['map_pose'])
                        if math.hypot(*actual) <= final_tolerance and n.arrival_heading_ok(samples[-1]['map_pose'],item['goal']):
                            self.valid_until = -math.inf
                            n.stopping = True
                            if not await n.release_controller(requests, results) or not await n.confirm_stop():
                                raise RuntimeError('Could not confirm final stop')
                            # A GNSS correction during braking must not fabricate completion.
                            latest = self.paired_samples(n.get_clock().now().nanoseconds/1e9)
                            final = latest[-1] if latest else None
                            if (final is not None and final['epoch'] == task_epoch and n.status(False)['ready']
                                    and n.arrival_heading_ok(final['map_pose'],item['goal'])
                                    and math.hypot(*body_delta(queue.target,final['map_pose'])) <= final_tolerance):
                                outcome = 'succeeded'
                                break
                    timings['state_and_hint_ms'] = (time.monotonic()-state_started)*1000
                    stage = time.monotonic()
                    if n.terrain is not None:
                        was_climbing = n.terrain.result['phase'] == 'climb'
                        n.terrain.tick()
                        if was_climbing != (n.terrain.result['phase'] == 'climb'):
                            # Stair suppression changes only on a climb transition.
                            terrain_stamp = n.get_clock().now().nanoseconds/1e9
                            self.valid_until = -math.inf
                    motion_allowed = (n.gait_confirmed(n.tasks.snapshot(task, include_paths=False))
                                      and (n.terrain is None or n.terrain.result['motion_allowed']))
                    controller_id = n.controller_id()
                    if not motion_allowed or (requests and n.active_controller != controller_id):
                        self.valid_until = -math.inf
                        self.active_result = None
                        n.stopping = True
                        n.command_pub.publish(Twist())
                        stage_release = time.monotonic()
                        if not await n.release_controller(requests, results):
                            raise RuntimeError('Could not release controller for terrain transition')
                        requests.clear()
                        results.clear()
                        timings['controller_release_ms'] = (time.monotonic()-stage_release)*1000
                    timings['fsm_ms'] = (time.monotonic()-stage)*1000
                    if not motion_allowed:
                        # Alignment and gait handshakes intentionally make no
                        # translational progress and do not consume this budget.
                        budget.last_progress_s=time.monotonic()
                        controller_attempted=False
                        # Alignment uses the live local map. A blocked FSM does
                        # not need to wait for a global planning map at all.
                        self.valid_until = boundary
                        self.report = dict(enabled=True,ready=self.allowed(),cycle=self.number,
                                           action=n.terrain.result['phase'] if n.terrain is not None else 'gait_confirmation',timings_ms=timings,
                                           compute_ms=(time.monotonic()-started)*1000)
                        self.history.append(self.report)
                        budget.planned(True,time.monotonic())
                        continue
                    stage = time.monotonic()
                    oldest, snapshot = self.grid(position, ros_now, terrain_stamp, observations)
                    timings['fusion_ms'] = (time.monotonic()-stage)*1000
                    ack_started = time.monotonic()
                    age = n.config['sensor_timeout_s']
                    remaining = oldest+age-n.get_clock().now().nanoseconds/1e9
                    deadline = min(boundary, ack_started+remaining)
                    if ack_started >= deadline:
                        raise TimeoutError('Planning snapshot expired during fusion')
                    token = snapshot.header.stamp.sec*10**9+snapshot.header.stamp.nanosec
                    self.global_ack = None
                    self.pub.publish(snapshot)
                    while self.global_ack != token:
                        if n.interrupted(task):
                            raise InterruptedError('Canceled waiting for planning costmap')
                        if time.monotonic() >= deadline:
                            raise TimeoutError('Planning costmap did not apply the snapshot before expiry')
                        await n.pause(.01)
                    timings['fusion_and_costmap_ms'] = (time.monotonic()-stage)*1000
                    timings['costmap_wait_ms'] = (time.monotonic()-ack_started)*1000
                    stage = time.monotonic()
                    if controller_attempted and not budget.controller_ended(stage):
                        raise RuntimeError('No odom progress within controller_patience_s')
                    final_yaw = None
                    if queue.final and 'yaw' in item['goal']:
                        final_yaw = item['goal']['yaw'] + position['yaw'] - samples[-1]['map_pose']['yaw']
                    path = await self.plan(position, hint, task, deadline, final_yaw)
                    timings['planning_ms'] = (time.monotonic()-stage)*1000
                    stage = time.monotonic()
                    command = FollowPath.Goal()
                    command.path, command.controller_id = path, controller_id
                    command.goal_checker_id = 'goal_checker'
                    handoff_started = True
                    pending = n.follow_client.send_goal_async(command)
                    requests.append(pending)
                    cycle_number = self.number
                    def accepted(f, number=cycle_number, deadline=deadline):
                        if not f.cancelled() and f.exception() is None:
                            h = f.result()
                            if h is not None and h.accepted and (n.interrupted(task) or number != self.number or time.monotonic() >= deadline):
                                h.cancel_goal_async()
                    pending.add_done_callback(accepted)
                    follower = await self.future(pending, task, deadline)
                    if not follower.accepted:
                        raise ValueError('Controller rejected cycle path')
                    results[pending] = follower.get_result_async()
                    self.active_result = results[pending]
                    n.active_controller = command.controller_id
                    timings['controller_accept_ms'] = (time.monotonic()-stage)*1000
                    remaining_age = oldest+age-n.get_clock().now().nanoseconds/1e9
                    if remaining_age <= 0:
                        raise TimeoutError('Planning snapshot expired before handoff')
                    # Allow the accepted path through the following refresh.
                    # Live obstacle data and the raw command watchdog still
                    # gate every command independently of this retry deadline.
                    self.valid_until = boundary + self.p['period_s']
                    if not self.allowed():
                        raise TimeoutError('Cycle result or source data expired before use')
                    n.stopping = False
                    n.path_pub.publish(path)
                    n.tasks.update(task,state='navigating', execution_path=[{'x':p.pose.position.x,'y':p.pose.position.y} for p in path.poses],
                                   path_revision=self.number, execution_frame='odom', hint_index=queue.index,
                                   hint_count=len(queue.indices), distance_remaining_m=math.hypot(x,y),reason=None)
                    self.report = dict(enabled=True,ready=True,cycle=self.number,hint_map=queue.target,
                        hint_odom=hint,hint_index=queue.index,hint_count=len(queue.indices),
                        sample_count=len(samples),period_s=self.p['period_s'],timings_ms=timings,
                        compute_ms=(time.monotonic()-started)*1000)
                    self.history.append(self.report)
                    budget.planned(True,time.monotonic())
                    controller_attempted=True
                    # Retain only active goals for cancellation; completed handles cannot grow unbounded.
                    requests[:] = [r for r in requests if r not in results or not results[r].done()]
                    results = {r:f for r,f in results.items() if r in requests}
                except (ValueError, TimeoutError) as exc:
                    # A delayed planning refresh is not a new obstacle or a
                    # lost source. Keep an already accepted path under the live
                    # controller/watchdog gates, within its original deadline.
                    can_retry = isinstance(exc, (SnapshotUnavailable, TimeoutError))
                    if can_retry and not handoff_started and not n.stopping and self.allowed():
                        self.report = dict(enabled=True, ready=True, cycle=self.number,
                                           action='keep_active_path', reason=str(exc), timings_ms=timings,
                                           compute_ms=(time.monotonic()-started)*1000)
                        self.history.append(self.report)
                        continue
                    self.valid_until = -math.inf
                    n.stopping = True
                    n.command_pub.publish(Twist())
                    for request in requests:
                        if request.done() and not request.cancelled():
                            accepted_goal = request.result()
                            if accepted_goal is not None and accepted_goal.accepted:
                                accepted_goal.cancel_goal_async()
                    self.report = dict(enabled=True,ready=False,cycle=self.number,reason=str(exc),
                                       timings_ms=timings,compute_ms=(time.monotonic()-started)*1000)
                    self.history.append(self.report)
                    n.tasks.update(task,state='planning',reason=str(exc))
                    if not budget.planned(False,time.monotonic()):
                        raise RuntimeError('Decision cycle retry budget exhausted: '+str(exc)) from exc
            if n.interrupted(task):
                outcome, reason = 'canceled', 'Task canceled'
        except InterruptedError as exc:
            outcome, reason = 'canceled', str(exc)
        except Exception as exc:
            reason = str(exc)
        finally:
            if n.owner == task:
                self.valid_until = -math.inf
                self.active_result = None
                self.report={**self.report,'ready':False,'task_outcome':outcome,'reason':reason}
                n.stopping = True
                released = await n.release_controller(requests, results)
                stopped = await n.confirm_stop()
                if not released or not stopped:
                    outcome, reason = 'failed', 'Cycle executor could not confirm controller release and stop'
                    n.stop_failure = reason
                    n.stop_recovery = (requests, results)
                self.report.update(task_outcome=outcome,reason=reason)
                n.tasks.update(task,state=outcome,reason=reason,completion_pose=n.pose())
                n.owner = None
            else:
                n.tasks.update(task,state=outcome,reason=reason)
            n.handles.pop(task,None); n.requests.pop(task,None); n.cancel_sent.discard(task)
            if handle.is_cancel_requested:handle.canceled()
            elif outcome == 'succeeded':handle.succeed()
            else:handle.abort()
        return NavigateToPose.Result()

    def grid(self, position, now, since=-math.inf, observations=None):
        n, lidar, camera, stamps = self.n, None, None, []
        age = n.config['sensor_timeout_s']
        if observations is None:
            with self.input_lock:
                observations = tuple(self.scans), self.camera
        scans, camera_msg = observations
        if n.config['lidar_enabled']:
            scan, tf = None, None
            for candidate in reversed(scans):
                stamp = acquisition_time(candidate)
                if stamp < since or stamp > now or not healthy_scan(candidate, now, age):
                    continue
                try:
                    tf = n.tf.lookup_transform('odom', candidate.header.frame_id,
                                               Time.from_msg(candidate.header.stamp))
                except TransformException:
                    continue
                scan = candidate
                break
            if scan is None:
                raise SnapshotUnavailable('Waiting for a fresh lidar snapshot with acquisition-time TF')
            stamp = acquisition_time(scan)
            q,t = tf.transform.rotation,tf.transform.translation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            rays=[(yaw+scan.angle_min+i*scan.angle_increment,float(r)) for i,r in enumerate(scan.ranges)
                  if math.isinf(r) and r>0 or math.isfinite(r) and scan.range_min<=r<=scan.range_max]
            if not rays:raise ValueError('Lidar snapshot has no valid rays')
            lidar=((t.x,t.y),rays,scan.range_max);stamps.append(stamp)
        if n.config['camera_enabled']:
            msg=camera_msg
            if msg is None or msg.header.frame_id!='odom':
                raise SnapshotUnavailable('Waiting for odom camera snapshot')
            stamp=acquisition_time(msg)
            camera=(np.asarray(msg.data).reshape(msg.info.height,msg.info.width),
                    (msg.info.origin.position.x,msg.info.origin.position.y),msg.info.resolution)
            if not np.any(camera[0]>=0):raise ValueError('Camera snapshot contains no observations')
            stamps.append(stamp)
        if not stamps or any(s < since or not 0 <= now-s < age for s in stamps):
            raise SnapshotUnavailable('No fresh obstacle snapshot for the current terrain mode')
        values,origin=fuse_grid((position['x'],position['y']),self.p['grid_resolution_m'],self.p['grid_half_width_m'],lidar,camera)
        msg=OccupancyGrid()
        msg.header.frame_id='odom';msg.header.stamp=n.get_clock().now().to_msg()
        msg.info.resolution=self.p['grid_resolution_m'];msg.info.height,msg.info.width=values.shape
        msg.info.origin.position.x,msg.info.origin.position.y=map(float,origin)
        msg.info.origin.orientation.w=1.;msg.data=array('b',values.astype(np.int8).tobytes())
        return min(stamps),msg

    async def plan(self, position, hint, task, deadline, final_yaw=None):
        dx,dy=hint['x']-position['x'],hint['y']-position['y']
        scale=min(1.,self.p['lookahead_m']/max(math.hypot(dx,dy),1e-9))
        request=ComputePathToPose.Goal()
        for pose,x,y in ((request.start,position['x'],position['y']),
                         (request.goal,position['x']+dx*scale,position['y']+dy*scale)):
            pose.header.frame_id='odom';pose.header.stamp=self.n.get_clock().now().to_msg()
            pose.pose.position.x,pose.pose.position.y=float(x),float(y)
            pose.pose.orientation.w=1.
        request.start.pose.orientation.z=math.sin(position['yaw']/2)
        request.start.pose.orientation.w=math.cos(position['yaw']/2)
        heading = final_yaw if final_yaw is not None and scale == 1. else math.atan2(dy,dx)
        request.goal.pose.orientation.z=math.sin(heading/2)
        request.goal.pose.orientation.w=math.cos(heading/2)
        request.use_start=True
        pending=self.n.path_client.send_goal_async(request)
        def late(f):
            try:
                h=f.result()
                if h.accepted and (time.monotonic() >= deadline or self.n.interrupted(task)):
                    h.cancel_goal_async()
            except Exception:
                pass
        pending.add_done_callback(late)
        goal=None
        try:
            goal=await self.future(pending,task,deadline)
            if not goal.accepted:raise ValueError('Planner rejected heuristic goal')
            result=await self.future(goal.get_result_async(),task,deadline)
            if result.status!=4 or not result.result.path.poses:
                raise ValueError('No sensor-traversable path toward hint')
            if len(result.result.path.poses)==1:
                result.result.path.poses=[request.start,request.goal]
            return result.result.path
        finally:
            if goal is not None and goal.accepted:goal.cancel_goal_async()
