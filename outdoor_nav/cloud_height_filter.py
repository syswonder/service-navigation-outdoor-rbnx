"""Acquisition-time base-frame height envelope, then local terrain arbitration."""
import copy
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from .configuration import load_runtime
from .terrain_inputs import TerrainInputs, rotation
from .core.terrain_geometry import height_keep, suppress_stair_points


def apply(points, transform):
    t = transform.transform.translation
    return points @ rotation(transform.transform.rotation).T + np.array([t.x, t.y, t.z])


class CloudHeightFilter(Node):
    def __init__(self, runtime):
        super().__init__('outdoor_cloud_height_filter')
        self.p, self.projection = runtime['terrain'], runtime['scan_projection']
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.inputs = TerrainInputs(self, runtime) if self.p['enabled'] else None
        self.timeout = runtime['policy']['sensor_timeout_s']
        self.pub = self.create_publisher(PointCloud2, runtime['topic_remap']['filtered_cloud'], 10)
        self.create_subscription(PointCloud2, runtime['topic_remap']['cloud'], self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        try:
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if not msg.header.frame_id or not 0 < stamp <= self.get_clock().now().nanoseconds/1e9 or self.get_clock().now().nanoseconds/1e9-stamp >= self.timeout:
                return
            values = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.column_stack([values[n] for n in ('x', 'y', 'z')]).reshape(-1, 3)
            if not len(points):
                return  # Empty raw input cannot certify free space.
            if msg.header.frame_id != 'base_link':
                tf = self.tf.lookup_transform('base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp), timeout=Duration(seconds=.05))
                points = apply(points, tf)
            if self.p['slope_filter_enabled'] and self.p['lidar_slope_enabled']:
                keep = height_keep(points, self.p)
            else:
                keep = np.isfinite(points).all(axis=1) & (points[:, 2] >= self.projection['min_height_m']) & (points[:, 2] <= self.projection['max_height_m'])
            points = points[keep]
            if self.inputs and len(points):
                obs, state = self.inputs.suppression_context()
                if obs and state.get('phase') == 'climb':
                    tf = self.tf.lookup_transform('odom', 'base_link', Time.from_msg(msg.header.stamp), timeout=Duration(seconds=.05))
                    remove = suppress_stair_points(apply(points, tf), obs, state, 'lidar_obstacles', self.p)
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
