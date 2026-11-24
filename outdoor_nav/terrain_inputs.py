"""ROS input validation shared by the executor and source-specific filters."""
import json
import math
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from .core.terrain import TerrainMachine


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class TerrainInputs:
    def __init__(self, node, runtime, control=False):
        self.node, self.p, self.t = node, runtime['terrain'], runtime['topic_remap']
        self.observations, self.state = {}, {}
        cycle = runtime.get('decision_cycle', {})
        self.state_timeout = max(self.p['observation_timeout_s'],
                                 cycle['period_s'] if cycle.get('enabled') else 0.)
        self.pitch, self.pitch_stamp = None, -math.inf
        self.visual_required = runtime['policy']['camera_enabled'] and runtime['semantic_segmentation']['enabled'] and bool(self.p['stairs_classes'])
        self.sources = []
        if self.visual_required:
            self.sources.append(('visual_semantics', 'visual_terrain'))
        # TODO: lidar semantics producer and integration; no subscription yet.
        for source, role in self.sources:
            node.create_subscription(String, self.t[role], lambda msg, s=source: self.receive(msg, s), 5)
        if control:
            node.create_subscription(Imu, self.t['imu'], self.imu, qos_profile_sensor_data)
            self.pub = node.create_publisher(String, self.t['terrain_state'], 5)
            self.machine = TerrainMachine({**self.p, 'gait_switch_timeout_s': runtime['policy']['gait_switch_timeout_s']})
        else:
            node.create_subscription(String, self.t['terrain_state'], self.state_message, 5)

    def now(self):
        return self.node.get_clock().now().nanoseconds/1e9

    def fresh(self, stamp, timeout=None):
        now = self.now()
        age = self.p['observation_timeout_s'] if timeout is None else timeout
        return type(stamp) in (int, float) and math.isfinite(stamp) and 0 < stamp <= now and now-stamp < age

    def receive(self, msg, source):
        if source not in {s for s, _ in self.sources}:
            return
        try:
            if len(msg.data) > 1000000:
                raise ValueError('Terrain message too large')
            value = json.loads(msg.data)
            if not isinstance(value, dict) or value.get('schema_version') != 1 or value.get('valid') is not True or value.get('frame_id') != 'odom' or not self.fresh(value.get('stamp')):
                raise ValueError('Invalid terrain observation')
            previous = self.observations.get(source)
            if previous and value['stamp'] <= previous['stamp']:
                return
            points = np.asarray(value.get('points', []), float).reshape(-1, 3)
            heading = value.get('heading_yaw')
            front_m = value.get('front_m')
            if (len(points) > 4096 or not np.isfinite(points).all()
                    or type(value.get('stairs')) is not bool
                    or value.get('direction') not in ('up', 'unknown')
                    or (front_m is not None and (type(front_m) not in (int, float) or not math.isfinite(front_m) or front_m < 0))
                    or (heading is not None and (type(heading) not in (int, float) or not math.isfinite(heading)))):
                raise ValueError('Invalid terrain payload')
            bounds = value.get('flight_bounds')
            if bounds is not None and (not isinstance(bounds, dict)
                    or any(not isinstance(xy, list) or len(xy) != 2
                           or any(type(v) not in (int, float) or not math.isfinite(v) for v in xy)
                           for xy in (bounds.get('front_xy'), bounds.get('end_xy')))):
                raise ValueError('Invalid stair flight bounds')
            self.observations[source] = {**value, 'source': source}
        except (ValueError, TypeError, KeyError):
            self.observations.pop(source, None)

    def selected(self, state='flat'):
        enabled = {source for source, _ in self.sources}
        available = [v for source, v in self.observations.items()
                     if source in enabled and self.fresh(v['stamp'])]
        # A missing required source is unknown, not an assertion that stairs ended.
        if len(available) != len(self.sources):
            return None
        if not self.sources:
            return dict(valid=True, stairs=False, stamp=self.now(), points=[])
        ranks = self.p['priorities']['stairs_up' if state == 'stairs_up' else 'flat']
        return max(available, key=lambda v: (ranks[v['source']], v['stairs']))

    def state_message(self, msg):
        try:
            value = json.loads(msg.data)
            self.state = value if isinstance(value, dict) and self.fresh(value.get('stamp'), self.state_timeout) else {}
        except (ValueError, TypeError):
            self.state = {}

    def suppression_context(self):
        state = self.state if self.fresh(self.state.get('stamp'), self.state_timeout) else {}
        return self.selected(state.get('state')), state

    def imu(self, msg):
        try:
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            q = msg.orientation
            if (not self.fresh(stamp) or msg.orientation_covariance[0] < 0
                    or not msg.header.frame_id or abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1) > .01):
                raise ValueError('Invalid IMU orientation')
            tf = self.node.tf.lookup_transform(msg.header.frame_id, 'base_link', Time.from_msg(msg.header.stamp), timeout=Duration(seconds=.05))
            matrix = rotation(q) @ rotation(tf.transform.rotation)
            pitch = math.asin(float(np.clip(-matrix[2, 0], -1., 1.)))
            self.pitch = self.p['pitch_sign']*pitch + math.radians(self.p['pitch_offset_deg'])
            self.pitch_stamp = stamp
        except Exception:
            self.pitch = None
