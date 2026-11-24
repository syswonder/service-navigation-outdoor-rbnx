"""Registered depth + interchangeable model labels -> acquisition-stamped terrain."""
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'vendor'))
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_sanity_checker.images import ImageDecoder
from semantic_local_grid.geometry import transform
from semantic_local_grid.ros_utils import matrix_from_tf, seconds
from .configuration import load_runtime
from .core.terrain_geometry import forward_mask, stair_heading


class TerrainPerception(Node):
    def __init__(self, runtime):
        super().__init__('outdoor_terrain_perception')
        self.p = runtime['terrain']
        self.range = runtime['policy']['camera_max_range_m']
        self.ids = [int(k) for k, v in runtime['semantic_segmentation']['classes'].items()
                    if v in self.p['stairs_classes']]
        t = runtime['topic_remap']
        self.pub = self.create_publisher(String, t['visual_terrain'], 5)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.decoder = ImageDecoder()
        self.last_stamp = 0.
        # Labels already mark low-confidence pixels unknown; only geometry is paired here.
        self.subscribers = [Subscriber(self, kind, t[name], qos_profile=qos_profile_sensor_data)
                            for kind, name in ((Image, 'camera_labels'), (Image, 'camera_depth'),
                                               (CameraInfo, 'camera_depth_info'),
                                               (CameraInfo, 'camera_color_info'))]
        # The colour and depth streams are free-running and land ~0.2 s apart,
        # so a 50 ms slop never pairs them; stair labels are then projected
        # onto the nearest depth frame instead.
        self.sync = ApproximateTimeSynchronizer(self.subscribers, 30, .30)
        self.sync.registerCallback(self.receive)

    def receive(self, labels_msg, depth_msg, di, ci):
        stamp = seconds(depth_msg.header.stamp)
        result = dict(schema_version=1, stamp=stamp, frame_id='odom', valid=False,
                      stairs=False, direction='unknown', heading_yaw=None, points=[])
        try:
            now = self.get_clock().now().nanoseconds/1e9
            if not 0 < stamp <= now or now-stamp >= self.p['observation_timeout_s'] or stamp <= self.last_stamp:
                raise ValueError('Stale/repeated terrain image')
            messages = (labels_msg, depth_msg, di, ci)
            if (len({m.header.frame_id for m in messages}) != 1 or not depth_msg.header.frame_id
                    or len({(m.width, m.height) for m in messages}) != 1
                    or not np.allclose(di.k, ci.k) or di.k[0] <= 0 or di.k[4] <= 0
                    # Only the registered depth frame must be rectified; the
                    # colour intrinsics keep the lens distortion of the raw
                    # image the labels were computed on.
                    or any(m.binning_x > 1 or m.binning_y > 1 or m.roi.x_offset or m.roi.y_offset
                           or np.any(np.abs(m.d) > 1e-8) or not np.allclose(m.r, np.eye(3).ravel()) for m in (di,))):
                raise ValueError('Terrain requires rectified pixel-aligned RGB/depth/calibration')
            if labels_msg.encoding != 'mono16' or depth_msg.encoding not in ('16UC1', '32FC1'):
                raise ValueError('Invalid terrain image encodings')
            labels = self.decoder.imgmsg_to_cv2(labels_msg, 'passthrough')[::3, ::3]
            depth = self.decoder.imgmsg_to_cv2(depth_msg, 'passthrough')[::3, ::3].astype(float)
            if depth_msg.encoding == '16UC1':
                depth *= .001
            v, u = np.indices(depth.shape)*3
            optical = np.stack(((u-di.k[2])*depth/di.k[0], (v-di.k[5])*depth/di.k[4], depth), -1).reshape(-1, 3)
            # The depth stamp can sit a few tens of milliseconds ahead of the
            # latest published transform; the newest available transform is
            # accurate enough at walking speed and keeps frames from dropping.
            base_tf = self.tf.lookup_transform('base_link', depth_msg.header.frame_id,
                                              Time(), timeout=Duration(seconds=.1))
            base = transform(optical, matrix_from_tf(base_tf))
            valid = forward_mask(base, self.range, self.p['roi_half_width_m'])
            valid &= (depth.ravel() > .2) & (labels.ravel() != 65535)
            if np.count_nonzero(valid) < self.p['min_stair_points']:
                raise ValueError('Insufficient classified depth in forward terrain ROI')
            points = base[valid & np.isin(labels.ravel(), self.ids)]
            result.update(stair_heading(points, self.p))
            result['front_m'] = float(np.min(np.linalg.norm(points[:, :2], axis=1))) if len(points) else None
            odom_tf = self.tf.lookup_transform('odom', 'base_link', Time(), timeout=Duration(seconds=.1))
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
            result['valid'] = True
            self.last_stamp = stamp
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
