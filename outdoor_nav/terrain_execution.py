"""Terrain supervision inside the existing single velocity owner."""
import json
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt32
from rclpy.time import Time
from .terrain_inputs import TerrainInputs
from .core.gait import chassis_prepared


def rotation_clear(n):
    grid = n.local_map
    if grid is None or time.monotonic()-n.stats.get('local_map', -math.inf) >= n.config['sensor_timeout_s']:
        return False
    try:
        if grid.header.frame_id != 'odom' or grid.info.resolution <= 0:
            return False
        tf = n.tf.lookup_transform('odom', 'base_link', Time())
        t = tf.transform.translation
        # Retain the branch's rotation check with a smaller experimental
        # stair clearance; this is not a measured chassis sweep radius.
        stairs = bool((self.observation or {}).get('stairs'))
        r = 0.25 if stairs else n.config['footprint_radius_m']
        meta = grid.info
        # Full circumscribed disk must be observed and free of lethal costs.
        low_x = math.floor((t.x-r-meta.origin.position.x)/meta.resolution)
        high_x = math.ceil((t.x+r-meta.origin.position.x)/meta.resolution)
        low_y = math.floor((t.y-r-meta.origin.position.y)/meta.resolution)
        high_y = math.ceil((t.y+r-meta.origin.position.y)/meta.resolution)
        if low_x < 0 or low_y < 0 or high_x >= meta.width or high_y >= meta.height:
            return False
        for y in range(low_y, high_y+1):
            for x in range(low_x, high_x+1):
                dx = meta.origin.position.x+(x+.5)*meta.resolution-t.x
                dy = meta.origin.position.y+(y+.5)*meta.resolution-t.y
                if math.hypot(dx, dy) <= r+meta.resolution:
                    cost = grid.data[y*meta.width+x]
                    if cost < 0 or cost >= 99:
                        return False
        return True
    except Exception:
        return False


class TerrainExecution:
    def __init__(self, node):
        self.node = node
        self.inputs = TerrainInputs(node, node.runtime, control=True)
        self.result = self.inputs.machine.output()
        self.at = -math.inf
        self.task_id = None
        self.prepared_task = None

    def tick(self):
        n, p, i = self.node, self.inputs.p, self.inputs
        task = n.tasks.snapshot(n.owner, include_paths=False) if n.owner else None
        enable = bool(n.service_active and n.config['motion_enabled'] and task
                      and task['task_id'] == self.prepared_task
                      and not task['cancel_requested'] and task['state'] in ('planning', 'navigating', 'replanning'))
        task_id = task['task_id'] if task else None
        if task_id != self.task_id:
            i.machine.reset()
            self.task_id = task_id
        now = time.monotonic()
        body = n.chassis
        age = body.get('motion_feedback_age_s')
        fresh = (now-n.chassis_at < n.config['sensor_timeout_s'] and type(age) in (float, int)
                 and math.isfinite(age) and 0 <= age < n.config['sensor_timeout_s'])
        ready = chassis_prepared(body, now-n.chassis_at)
        stopped = (n.stationary_since is not None and now-n.stationary_since >= n.config['stop_hold_s']
                   and now-n.velocity_at < n.config['sensor_timeout_s'])
        position = yaw = None
        try:
            tf = n.tf.lookup_transform('odom', 'base_link', Time())
            q = tf.transform.rotation
            stamp = tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
            if i.fresh(stamp):
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                t = tf.transform.translation
                if math.isfinite(t.x) and math.isfinite(t.y):
                    position = t.x, t.y
        except Exception:
            pass
        self.observation = i.selected(i.machine.state)
        self.result = i.machine.tick(now, enable=enable,
            pitch=i.pitch if i.fresh(i.pitch_stamp) else None,
            observation=self.observation, yaw=yaw, position=position,
            actual_gait=body.get('gait'), gait_fresh=fresh, gait_ready=ready, stopped=stopped)
        self.at = now
        request = self.result['gait_request']
        # The existing calibration/localization/ownership gates also guard gait requests.
        if request is not None and n.gait_pub is not None and n.status(False)['ready']:
            n.gait_pub.publish(UInt32(data=request))
        i.pub.publish(String(data=json.dumps({**self.result, 'stamp': i.now(), 'schema_version': 1})))
        if task and task.get('terrain_status') != self.result:
            n.tasks.update(task_id, terrain_status=self.result)

    def command(self, safe, command):
        n, result = self.node, self.result
        out = Twist()
        cyclic = n.runtime.get('decision_cycle', {}).get('enabled', False)
        if not safe or (not cyclic and time.monotonic() - self.at >= n.config['command_timeout_s']):
            return out
        if result['angular_z'] and result['phase'] == 'align':
            # Rotation sweeps the footprint; check the actual fused local costmap.
            if self.rotation_clear():
                out.angular.z = result['angular_z']
            return out
        if result['motion_allowed']:
            if result['state'] == 'stairs_up':
                if getattr(n, 'active_controller', None) != 'StairsFollowPath':
                    return out
                out.linear.x = min(max(0., command.linear.x), self.inputs.p['stairs_speed_mps'])
                out.angular.z = command.angular.z
            else:
                if getattr(n, 'active_controller', None) != 'FollowPath':
                    return out
                return command
        return out

    def rotation_clear(self):
        return rotation_clear(self.node)
