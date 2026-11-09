"""Registered RGB-D low-obstacle detector; isolated replay entry point."""
import json
import time
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String, Header
from tf2_ros import Buffer, TransformListener
import message_filters
from sensor_sanity_checker.images import ImageDecoder
from semantic_local_grid.geometry import observations, transform
from semantic_local_grid.ros_utils import matrix_from_tf, cloud_message
from .core import detect_low_obstacles
from .geometric_perception import geometric_observation
from .rtabmap_backend import FrameSlot, fuse_rtabmap
from sensor_msgs_py import point_cloud2


class DepthObstacles(Node):

    def __init__(self):
        super().__init__('direct_depth_obstacles')
        self.declare_parameter('allow_live_shadow', False)
        self.declare_parameter('allow_outdoor_navigation', False)
        outdoor = self.get_parameter('allow_outdoor_navigation').value
        if os.environ.get('ROS_DOMAIN_ID', '0') not in ('77', '177'):
            if not self.get_parameter('allow_live_shadow').value and not outdoor:
                raise RuntimeError('Live domain requires explicit diagnostic shadow mode')
            expected = '/outdoor/direct_nav/visual_obstacles' if outdoor else '/field_shadow/direct_nav/visual_obstacles'
            if self.resolve_topic_name('/direct_nav/visual_obstacles') != expected:
                raise RuntimeError('Live shadow observations must use the field_shadow namespace')
        defaults = dict(planning_frame='odom',
                        base_frame='base_link',
                        alignment_verified=False,
                        allow_unverified_offline=False,
                        min_height=.10,
                        max_age=1.0,
                        pixel_stride=3,
                        navigation_pose_topic='',
                        visual_obstacles_enabled=True,
                        geometry_primary=False,
                        depth_backend='legacy',
                        rtabmap_timeout_s=.2,
                        geometry_expected_ground_z=-.18,
                        geometry_ground_z_tolerance=.05)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p['depth_backend'] not in ('legacy', 'rtabmap'):
            raise ValueError('Unsupported depth backend')
        if self.p['depth_backend'] == 'rtabmap' and not self.p['geometry_primary']:
            raise ValueError('RTAB-Map requires geometry_primary')
        self.rtab_slot = FrameSlot(self.p['rtabmap_timeout_s'])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.navigation_pose = None
        if self.p['navigation_pose_topic']:
            from semantic_local_grid.navigation_pose import NavigationPose
            self.navigation_pose = NavigationPose(self, self.p['navigation_pose_topic'])
        self.decoder = ImageDecoder()
        self.cloud_pub = self.create_publisher(PointCloud2, '/direct_nav/visual_obstacles', 5)
        self.ground_pub = self.create_publisher(PointCloud2, '/direct_nav/observed_ground', 5)
        self.status_pub = self.create_publisher(String, '/direct_nav/depth_status', 5)
        if self.p['depth_backend'] == 'rtabmap':
            self.rtab_pub = self.create_publisher(PointCloud2, '/direct_nav/rtabmap/input', 1)
            self.rtab_subs = [self.create_subscription(
                PointCloud2, '/direct_nav/rtabmap/'+kind,
                lambda msg, k=kind: self.rtab_receive(k, msg), qos_profile_sensor_data)
                for kind in ('ground', 'obstacles')]
            self.rtab_timer = self.create_timer(.025, self.rtab_expire)
        topics = [(Image, '/camera_gemini330/depth/image_raw'),
                  (CameraInfo, '/camera_gemini330/depth/camera_info'),
                  (CameraInfo, '/camera_gemini330/color/camera_info')]
        if not self.p['geometry_primary']:
            topics.insert(0, (Image, '/semantic/road_mask'))
        self.subs = [
            message_filters.Subscriber(self,
                                       typ,
                                       topic,
                                       qos_profile=QoSProfile(
                                           depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            for typ, topic in topics
        ]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subs, 100, .05)
        self.sync.registerCallback(self.receive_geometry if self.p['geometry_primary'] else self.receive)

    def matrix(self, target, source, stamp):
        if target == source:
            return np.eye(4)
        if self.navigation_pose and target == self.p['planning_frame'] and source == self.p[
                'base_frame']:
            return self.navigation_pose.matrix(stamp,wait_timeout=.12)
        return matrix_from_tf(
            self.tf.lookup_transform(target,
                                     source,
                                     Time.from_msg(stamp),
                                     timeout=Duration(seconds=.1)))

    def receive_geometry(self, depth, info, color_info):
        return self.receive(None, depth, info, color_info)

    def rtab_expire(self):
        old = self.rtab_slot.expired(time.monotonic())
        if old:
            status = old['payload']['status']
            status.update(valid=False, reason='RTABMAP_TIMEOUT',
                          processing_s=time.monotonic()-old['payload']['started'])
            self.status_pub.publish(String(data=json.dumps(status)))

    def rtab_receive(self, kind, msg):
        self.rtab_expire()
        key = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        current = self.rtab_slot.pending
        if current is None or key != current['key']:
            return
        if msg.header.frame_id != self.p['base_frame']:
            self.rtab_slot.pending = None
            status = current['payload']['status']
            status.update(valid=False, reason='RTABMAP_FRAME_MISMATCH')
            self.status_pub.publish(String(data=json.dumps(status)))
            return
        try:
            values = point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True)
            points = np.column_stack([values[name] for name in ('x','y','z')]).astype(float)
            points[:, 2] += self.p['geometry_expected_ground_z']
            done = self.rtab_slot.add(key, kind, points, time.monotonic())
            if done is None:
                return
            data, outputs = done['payload'], done['outputs']
            status = data['status']
            ground, obstacles, result = fuse_rtabmap(outputs['ground'], outputs['obstacles'],
                data['raw_hazards'], data['reference_status'],
                self.p['geometry_expected_ground_z'], self.p['min_height'])
            status.update(result)
            status['rtabmap_roundtrip_s'] = time.monotonic()-done['started']
            status['skipped_busy_frames'] = self.rtab_slot.skipped
            self.publish_geometry(ground, obstacles, data['header'], status)
        except Exception as exc:
            self.rtab_slot.pending = None
            data = current['payload']
            status = data['status']
            status.update(valid=False, reason=str(exc))
        status['processing_s'] = time.monotonic()-data['started']
        status['output_age_s'] = self.get_clock().now().nanoseconds*1e-9-status['stamp']
        self.status_pub.publish(String(data=json.dumps(status)))

    def publish_geometry(self, ground, obstacles, source_header, status):
        planning = self.matrix(self.p['planning_frame'], self.p['base_frame'], source_header.stamp)
        if not 0 <= self.get_clock().now().nanoseconds*1e-9-status['stamp'] <= self.p['max_age']:
            raise ValueError('DEPTH_EXPIRED_DURING_PROCESSING')
        header = Header(stamp=source_header.stamp, frame_id=self.p['planning_frame'])
        self.cloud_pub.publish(cloud_message(transform(obstacles, planning), header))
        self.ground_pub.publish(cloud_message(transform(ground, planning), header))
        status['calibration_verified'] = self.p['alignment_verified']

    def receive(self, mask, depth, info, color_info):
        if self.p['depth_backend'] == 'rtabmap':
            self.rtab_expire()
            if self.rtab_slot.pending is not None:
                self.rtab_slot.skipped += 1
                return
        stamp = depth.header.stamp.sec + depth.header.stamp.nanosec * 1e-9
        processing_started = time.monotonic()
        status = {
            'valid': False,
            'stamp': stamp,
            'obstacle_detection_enabled': self.p['visual_obstacles_enabled'],
            'input_age_s': self.get_clock().now().nanoseconds * 1e-9 - stamp
        }
        try:
            if not self.p['alignment_verified'] and not self.p['allow_unverified_offline']:
                raise ValueError('CALIBRATION_UNVERIFIED')
            now = self.get_clock().now().nanoseconds * 1e-9
            stamps = [x.header.stamp.sec+x.header.stamp.nanosec*1e-9 for x in (depth, info, color_info)]
            if max(stamps)-min(stamps) > .05 or min(stamps) <= 0:
                raise ValueError('GEOMETRY_INPUTS_UNSYNCHRONIZED')
            if not 0 <= now - stamp <= self.p['max_age']:
                raise ValueError('DEPTH_STALE_OR_FUTURE')
            if (mask is not None and mask.header.frame_id != depth.header.frame_id) or info.header.frame_id != depth.header.frame_id:
                raise ValueError('REGISTERED_FRAMES_DIFFER')
            if color_info.header.frame_id != info.header.frame_id:
                raise ValueError('COLOR_DEPTH_FRAMES_DIFFER')
            if mask is not None and (mask.width, mask.height) != (depth.width, depth.height):
                raise ValueError('REGISTERED_DIMENSIONS_DIFFER')
            if (info.width, info.height) != (depth.width,
                                             depth.height) or not np.allclose(info.k, color_info.k):
                raise ValueError('REGISTERED_CALIBRATION_DIFFERS')
            if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
                raise ValueError('UNSUPPORTED_ROI_OR_BINNING')
            if np.any(np.abs(info.d) > 1e-8) or not np.allclose(info.r, np.eye(3).ravel()):
                raise ValueError('DEPTH_NOT_RECTIFIED')
            if (mask is not None and mask.encoding != 'mono8') or depth.encoding not in ('16UC1', '32FC1'):
                raise ValueError('UNSUPPORTED_ENCODING')
            d = self.decoder.imgmsg_to_cv2(depth, 'passthrough')
            m = (np.ones(d.shape, np.uint8) if mask is None else self.decoder.imgmsg_to_cv2(mask, 'passthrough'))
            optical, labels = observations(d, m, info.k, .001 if depth.encoding == '16UC1' else 1.,
                                           .2, 6., self.p['pixel_stride'])
            base = transform(
                optical, self.matrix(self.p['base_frame'], depth.header.frame_id,
                                     depth.header.stamp))
            if self.p['geometry_primary']:
                ground, obstacles, result = geometric_observation(
                    base, expected_ground_z=self.p['geometry_expected_ground_z'],
                    ground_z_tolerance=self.p['geometry_ground_z_tolerance'],
                    min_height=self.p['min_height'])
            else:
                obstacles, result = detect_low_obstacles(
                    base, labels, min_height=self.p['min_height'],
                    detect_obstacles=self.p['visual_obstacles_enabled'])
                ground = np.empty((0, 3))
                if result['valid']:
                    model = np.asarray(result['plane_abc'])
                    residual = base[:, 2]-np.column_stack((base[:, :2], np.ones(len(base))))@model
                    ground = base[(np.abs(residual) <= .018) & (base[:, 0] > .15)
                                  & (base[:, 0] < 5) & (np.abs(base[:, 1]) < 2.5)]
            status.update(result)
            if self.p['depth_backend'] == 'rtabmap':
                if self.rtab_pub.get_subscription_count() == 0:
                    raise ValueError('RTABMAP_NOT_CONNECTED')
                key = depth.header.stamp.sec*10**9+depth.header.stamp.nanosec
                shifted = base[(base[:,0]>-.8)&(base[:,0]<5)&(abs(base[:,1])<3)].copy()
                shifted[:,2] -= self.p['geometry_expected_ground_z']
                header = Header(stamp=depth.header.stamp, frame_id=self.p['base_frame'])
                payload = dict(status=status, reference_status=result, raw_hazards=obstacles,
                               header=depth.header, started=processing_started)
                if self.rtab_slot.begin(key, time.monotonic(), payload):
                    self.rtab_pub.publish(cloud_message(shifted, header))
                return
            if not result['valid']:
                raise ValueError(result['reason'])
            self.publish_geometry(ground, obstacles, depth.header, status)
        except Exception as exc:
            status.update(valid=False, reason=str(exc))
        status['processing_s'] = time.monotonic() - processing_started
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacles()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
