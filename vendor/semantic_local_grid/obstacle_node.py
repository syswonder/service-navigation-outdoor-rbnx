"""obstacle_projection_node: LiDAR PointCloud2 -> /obstacle/local_grid.

LiDAR only detects obstacles. It never clears free space and never classifies road,
so the obstacle grid is -1 UNKNOWN everywhere except 100 OBSTACLE cells. The point
type is read from the message fields at runtime (PointCloud2 is the only supported
type); any other topic type is rejected rather than assumed.
"""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener
from .ros_utils import matrix_from_tf, seconds
from .geometry import transform, rasterize
from .obstacle_geometry import filter_height, filter_range, voxel_downsample, OBS_UNKNOWN, OBS_OBSTACLE


class ObstacleNode(Node):
    def __init__(self, **kwargs):
        super().__init__('obstacle_projection_node', **kwargs)
        defaults = {
            'cloud_topic': '/scanner/cloud',
            'grid_topic': '/obstacle/local_grid',
            'planning_frame': '',
            'robot_base_frame': '',
            'resolution': 0.05, 'width': 160, 'height': 160,
            'origin_x': -2.0, 'origin_y': -4.0,
            'min_obstacle_height': 0.05,
            'max_obstacle_height': 2.0,
            'range_min': 0.0,
            'range_max': 8.0,
            'voxel_size': 0.1,
            'max_data_age': 1.0,
            'watchdog_timeout': 1.5,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        p = self.p
        if not p['planning_frame'] or not p['robot_base_frame']:
            raise ValueError('planning_frame and robot_base_frame are required')
        if p['resolution'] <= 0 or min(p['width'], p['height']) <= 0:
            raise ValueError('invalid grid dimensions')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.grid_pub = self.create_publisher(OccupancyGrid, p['grid_topic'], 5)
        self.status_pub = self.create_publisher(String, '/obstacle/status', 10)
        self.sub = self.create_subscription(PointCloud2, p['cloud_topic'], self.receive, qos_profile_sensor_data)
        self.last_success = None
        self.last_stamp = None
        self.processed = 0
        self.origin = np.array([p['origin_x'], p['origin_y']])
        self.yaw = 0.0
        self.timer = self.create_timer(0.25, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Obstacle projection ready')

    def matrix(self, target, source, stamp):
        if target == source:
            return np.eye(4)
        return matrix_from_tf(self.tf.lookup_transform(target, source, Time.from_msg(stamp), timeout=Duration(seconds=0.03)))

    def publish(self, grid, points, stamp):
        p = self.p
        header = Header(stamp=stamp, frame_id=p['planning_frame'])
        msg = OccupancyGrid()
        msg.header = header
        msg.info.map_load_time = stamp
        msg.info.resolution = p['resolution']
        msg.info.width = p['width']
        msg.info.height = p['height']
        msg.info.origin.position.x = float(self.origin[0])
        msg.info.origin.position.y = float(self.origin[1])
        msg.info.origin.orientation.z = float(np.sin(self.yaw / 2))
        msg.info.origin.orientation.w = float(np.cos(self.yaw / 2))
        msg.data = grid.reshape(-1).tolist()
        self.grid_pub.publish(msg)

    def fail(self, reason):
        self.publish(np.full((self.p['height'], self.p['width']), OBS_UNKNOWN, np.int8),
                     np.empty((0, 3)), self.get_clock().now().to_msg())
        self.status_pub.publish(String(data=json.dumps(
            {'ok': False, 'reason': reason, 'processed': self.processed, 'obstacle_cells': 0})))

    def watchdog(self):
        if self.last_success is None or time.monotonic() - self.last_success > self.p['watchdog_timeout']:
            self.fail('no fresh point cloud / watchdog expired')

    @staticmethod
    def read_points(msg):
        """Read finite (N,3) xyz float32 from a PointCloud2 without assuming the layout."""
        try:
            from sensor_msgs_py import point_cloud2
            pts = np.asarray(point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=False), dtype=np.float32).reshape(-1, 3)
        except Exception:
            if msg.point_step < 12:
                raise RuntimeError('cannot decode PointCloud2 point_step=%d' % msg.point_step)
            pts = np.frombuffer(msg.data, dtype='<f4').reshape(-1, msg.point_step // 4)[:, :3].astype(np.float32)
        return pts[np.isfinite(pts).all(axis=1)]

    def receive(self, msg):
        p = self.p
        try:
            stamp = seconds(msg.header.stamp)
            now = self.get_clock().now().nanoseconds * 1e-9
            if stamp <= 0 or now - stamp > p['max_data_age'] or stamp - now > 0.1:
                raise ValueError('stale, zero or future cloud timestamp')
            if self.last_stamp is not None and stamp <= self.last_stamp:
                self.last_stamp = None
                raise ValueError('repeated/backward cloud time')
            pts = self.read_points(msg)
            if len(pts) == 0:
                raise ValueError('empty point cloud')
            base_matrix = self.matrix(p['robot_base_frame'], msg.header.frame_id, msg.header.stamp)
            base_pts = transform(pts, base_matrix)
            base_pts = filter_height(base_pts, p['min_obstacle_height'], p['max_obstacle_height'])
            planning_matrix = self.matrix(p['planning_frame'], p['robot_base_frame'], msg.header.stamp)
            self.yaw = float(np.arctan2(planning_matrix[1, 0], planning_matrix[0, 0]))
            robot_xy = planning_matrix[:2, 3]
            pts = transform(base_pts, planning_matrix)
            pts = filter_range(pts, p['range_min'], p['range_max'], robot_xy)
            pts = voxel_downsample(pts, p['voxel_size'])
            labels = np.zeros(len(pts), dtype=np.uint8)
            grid, self.origin = rasterize(pts, labels, p['resolution'], p['width'], p['height'],
                                          p['origin_x'], p['origin_y'], robot_xy, self.yaw)
            self.publish(grid, pts, msg.header.stamp)
            self.last_success = time.monotonic()
            self.last_stamp = stamp
            self.processed += 1
            self.status_pub.publish(String(data=json.dumps({
                'ok': True, 'processed': self.processed,
                'raw_points': int(msg.width * msg.height),
                'filtered_points': int(len(pts)),
                'obstacle_cells': int((grid == OBS_OBSTACLE).sum()),
                'min_obstacle_height': p['min_obstacle_height'],
                'max_obstacle_height': p['max_obstacle_height'],
                'range_min': p['range_min'], 'range_max': p['range_max'],
                'voxel_size': p['voxel_size']})))

        except Exception as exc:
            self.fail(str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
