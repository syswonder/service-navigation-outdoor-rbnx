"""Registered depth + interchangeable model labels -> acquisition-stamped terrain."""
from collections import deque
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'vendor'))
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_sanity_checker.images import ImageDecoder
from semantic_local_grid.geometry import transform
from semantic_local_grid.ros_utils import matrix_from_tf, seconds
from .configuration import load_runtime
from .core.terrain_geometry import forward_mask, stair_front


class TerrainPerception(Node):
    def __init__(self, runtime):
        super().__init__('outdoor_terrain_perception')
        self.p = runtime['terrain']
        self.range = runtime['policy']['camera_max_range_m']
        self.threshold = runtime['semantic_segmentation']['confidence_threshold']
        self.ids = [int(k) for k, v in runtime['semantic_segmentation']['classes'].items()
                    if v in self.p['stairs_classes']]
        t = runtime['topic_remap']
        self.pub = self.create_publisher(String, t['visual_terrain'], 5)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.decoder = ImageDecoder()
        self.last_stamp = 0.
        self.last_now = 0.
        self.pending = deque(maxlen=30)
        self.subscribers = [Subscriber(self, kind, t[name], qos_profile=qos_profile_sensor_data)
                            for kind, name in ((Image, 'camera_labels'), (Image, 'camera_confidence'),
                                               (Image, 'camera_depth'), (CameraInfo, 'camera_depth_info'),
                                               (CameraInfo, 'camera_color_info'))]
        self.sync = ApproximateTimeSynchronizer(self.subscribers, self.pending.maxlen, .05)
        self.sync.registerCallback(self.receive)
        self.create_timer(.02, self.process_ready)

    def receive(self, labels_msg, confidence_msg, depth_msg, di, ci):
        stamp = seconds(depth_msg.header.stamp)
        if stamp > self.last_stamp:
            self.pending.append((stamp, (labels_msg, confidence_msg, depth_msg, di, ci)))

    def process_ready(self):
        now = self.get_clock().now().nanoseconds/1e9
        if now < self.last_now:
            self.pending.clear()
            self.last_stamp = 0.
        self.last_now = now
        timeout = self.p['observation_timeout_s']
        self.pending = deque(((stamp, messages) for stamp, messages in self.pending
                              if stamp > self.last_stamp and now-stamp < timeout),
                             maxlen=self.pending.maxlen)
        # TF is delivered by the same ROS executor. A blocking lookup in a
        # synchronizer callback prevents TF from catching up to the image.
        for stamp, messages in sorted(self.pending, key=lambda item: item[0], reverse=True):
            if stamp > now:
                continue
            depth = messages[2]
            if not depth.header.frame_id:
                self.process(stamp, messages, None, None)
                break
            capture_time = Time.from_msg(depth.header.stamp)
            try:
                base_tf = self.tf.lookup_transform('base_link', depth.header.frame_id, capture_time)
                odom_tf = self.tf.lookup_transform('odom', 'base_link', capture_time)
            except TransformException:
                continue
            self.process(stamp, messages, base_tf, odom_tf)
            break

    def process(self, stamp, messages, base_tf, odom_tf):
        self.last_stamp = stamp
        labels_msg, confidence_msg, depth_msg, di, ci = messages
        result = dict(schema_version=1, stamp=stamp, frame_id='odom', valid=False,
                      stairs=False, direction='unknown', heading_yaw=None, points=[])
        try:
            if (len({m.header.frame_id for m in messages}) != 1 or not depth_msg.header.frame_id
                    or len({(m.width, m.height) for m in messages}) != 1
                    or not np.allclose(di.k, ci.k) or di.k[0] <= 0 or di.k[4] <= 0
                    or any(m.binning_x > 1 or m.binning_y > 1 or m.roi.x_offset or m.roi.y_offset
                           or np.any(np.abs(m.d) > 1e-8) or not np.allclose(m.r, np.eye(3).ravel()) for m in (di, ci))):
                raise ValueError('Terrain requires rectified pixel-aligned RGB/depth/calibration')
            if labels_msg.encoding != 'mono16' or confidence_msg.encoding != '32FC1' or depth_msg.encoding not in ('16UC1', '32FC1'):
                raise ValueError('Invalid terrain image encodings')
            if labels_msg.header.stamp != confidence_msg.header.stamp:
                raise ValueError('Terrain labels and confidence must come from the same model frame')
            labels = self.decoder.imgmsg_to_cv2(labels_msg, 'passthrough')[::3, ::3]
            confidence = self.decoder.imgmsg_to_cv2(confidence_msg, 'passthrough')[::3, ::3]
            depth = self.decoder.imgmsg_to_cv2(depth_msg, 'passthrough')[::3, ::3].astype(float)
            if depth_msg.encoding == '16UC1':
                depth *= .001
            v, u = np.indices(depth.shape)*3
            optical = np.stack(((u-di.k[2])*depth/di.k[0], (v-di.k[5])*depth/di.k[4], depth), -1).reshape(-1, 3)
            base = transform(optical, matrix_from_tf(base_tf))
            valid = forward_mask(base, self.range, self.p['roi_half_width_m'])
            valid &= (depth.ravel() > .2) & np.isfinite(confidence.ravel()) & (confidence.ravel() >= self.threshold) & (labels.ravel() != 65535)
            if np.count_nonzero(valid) < self.p['min_stair_points']:
                raise ValueError('Insufficient classified depth in forward terrain ROI')
            points = base[valid & np.isin(labels.ravel(), self.ids)]
            result.update(stair_front(points, self.p))
            matrix = matrix_from_tf(odom_tf)
            if result['heading_yaw'] is not None:
                a = result['heading_yaw']
                normal = matrix[:3, :3] @ np.array([np.cos(a), np.sin(a), 0.])
                result['heading_yaw'] = float(np.arctan2(normal[1], normal[0]))
            # One sample per 5 cm voxel, bounded transport; do not send full RGB-D frames.
            if len(points):
                _, ids = np.unique(np.floor(points/.05).astype(int), axis=0, return_index=True)
                points = points[ids]
                points = points[::max(1, int(np.ceil(len(points)/2048)))]
            result['points'] = transform(points, matrix).tolist()
            if self.get_clock().now().nanoseconds/1e9-stamp >= self.p['observation_timeout_s']:
                raise ValueError('Terrain image expired during processing')
            result['valid'] = True
        except Exception as exc:
            result['reason'] = str(exc)
        self.pub.publish(String(data=json.dumps(result, allow_nan=False)))


def main():
    runtime = load_runtime(sys.argv[1])
    rclpy.init(args=sys.argv[2:])
    node = TerrainPerception(runtime)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
