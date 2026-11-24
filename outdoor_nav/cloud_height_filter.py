"""Acquisition-time base-frame height envelope, then local terrain arbitration."""
import copy
import sys
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from .configuration import load_runtime
from .terrain_inputs import TerrainInputs, rotation
from .core.terrain_geometry import height_keep, suppress_stair_points
from .core.height_filter import body_band, filter_xyz


def apply(points, transform):
    t = transform.transform.translation
    return points @ rotation(transform.transform.rotation).T + np.array([t.x, t.y, t.z])


class CloudHeightFilter(Node):
    def __init__(self, runtime):
        super().__init__('outdoor_cloud_height_filter')
        self.p, self.projection = runtime['terrain'], runtime['scan_projection']
        self.policy = runtime['policy']
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.inputs = TerrainInputs(self, runtime) if self.p['enabled'] else None
        self.timeout = runtime['policy']['sensor_timeout_s']
        self.pending = deque(maxlen=12)
        self.last_stamp = self.last_now = 0.
        self.pub = self.create_publisher(PointCloud2, runtime['topic_remap']['filtered_cloud'], 10)
        self.create_subscription(PointCloud2, runtime['topic_remap']['cloud'], self.receive, qos_profile_sensor_data)
        self.create_timer(.02, self.process_ready)

    def receive(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if msg.header.frame_id and stamp > 0:
            self.pending.append((stamp, msg))

    def process_ready(self):
        now = self.get_clock().now().nanoseconds/1e9
        if now < self.last_now:
            self.pending.clear()
            self.last_stamp = 0.
        self.last_now = now
        self.pending = deque(((stamp, msg) for stamp, msg in self.pending
                              if stamp > self.last_stamp and now-stamp < self.timeout), maxlen=self.pending.maxlen)
        obs, state = self.inputs.suppression_context() if self.inputs else (None, {})
        suppress = obs and obs.get('stairs') and state.get('phase') == 'climb'
        # Let this node's TF callbacks catch up instead of blocking them in receive().
        for stamp, msg in sorted(self.pending, key=lambda item: item[0], reverse=True):
            if stamp > now:
                continue
            try:
                at = Time.from_msg(msg.header.stamp)
                frame = msg.header.frame_id
                base_tf = self.tf.lookup_transform('base_link', frame, at) if frame != 'base_link' else None
                odom_tf = self.tf.lookup_transform('odom', 'base_link', at) if suppress else None
            except TransformException:
                continue
            self.last_stamp = stamp
            self.filter_cloud(msg, base_tf, odom_tf, obs, state)
            break

    def filter_cloud(self, msg, base_tf, odom_tf, obs, state):
        try:
            values = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.column_stack([values[n] for n in ('x', 'y', 'z')]).reshape(-1, 3)
            points = points[np.isfinite(points).all(axis=1)]
            if not len(points):
                return  # Empty raw input cannot certify free space.
            if base_tf is not None:
                points = apply(points, base_tf)
            if self.p['slope_filter_enabled'] and self.p['lidar_slope_enabled']:
                keep = height_keep(points, self.p)
            elif self.projection.get('fixed_height', False):
                keep = (np.isfinite(points).all(axis=1)
                        & (points[:, 2] >= self.projection['min_height_m'])
                        & (points[:, 2] <= self.projection['max_height_m']))
            else:
                p = self.policy
                ceiling = max(p['cloud_max_height_m'], p['cloud_min_height_far_m'] + .2)
                near, far, ceiling = body_band(p['cloud_min_height_near_m'],
                    p['cloud_min_height_far_m'], ceiling, -self.p['ground_z_m'])
                keep = filter_xyz(*points.T, near, far, p['cloud_height_ramp_m'], ceiling)
            points = points[keep]
            if odom_tf is not None and len(points):
                remove = suppress_stair_points(apply(points, odom_tf), obs, state, 'lidar_obstacles', self.p)
                points = points[~remove]
            header = copy.deepcopy(msg.header)
            header.frame_id = 'base_link'
            self.pub.publish(point_cloud2.create_cloud_xyz32(header, points.astype(np.float32)))
        except Exception as exc:
            self.get_logger().warning('Cloud terrain filter: ' + str(exc), throttle_duration_sec=5.)


def main():
    runtime = load_runtime(sys.argv[1])
    rclpy.init(args=sys.argv[2:])
    node = CloudHeightFilter(runtime)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
