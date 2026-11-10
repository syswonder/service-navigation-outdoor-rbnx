"""Filter a point cloud by a distance-graded minimum height before scan projection."""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

from .core.height_filter import body_band, filter_xyz


def rotation(transform):
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class CloudHeightFilter(Node):
    def __init__(self):
        super().__init__("outdoor_cloud_height_filter")
        self.near = float(self.declare_parameter("near_m", 0.2).value)
        self.far = float(self.declare_parameter("far_m", 0.5).value)
        self.ramp = float(self.declare_parameter("ramp_m", 8.0).value)
        self.ceiling = float(self.declare_parameter("max_m", 0.7).value)
        self.lidar_height = float(self.declare_parameter("lidar_height_m", 0.42).value)
        if not (0 <= self.near <= self.far < self.ceiling
                and self.ramp > 0 and self.lidar_height > 0):
            raise ValueError("Invalid graded height parameters")
        # A ceiling below the far threshold would delete every far point,
        # including real obstacles, so keep a thin band above either value.
        requested = self.ceiling
        self.ceiling = max(self.ceiling, self.far + 0.2)
        if self.ceiling != requested:
            self.get_logger().warn(
                "cloud_max_height_m %.2f is below the far height threshold; "
                "using %.2f for far points" % (requested, self.ceiling)
            )
        self.buffer = Buffer(cache_time=Duration(seconds=2.0))
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointCloud2, "/outdoor/cloud_for_scan", 10)
        self.create_subscription(PointCloud2, "cloud_in", self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        offsets, datatypes = {}, {}
        for field in msg.fields:
            if field.name in ("x", "y", "z"):
                offsets[field.name], datatypes[field.name] = field.offset, field.datatype
        if len(offsets) != 3 or msg.is_bigendian:
            return
        if any(kind != PointField.FLOAT32 for kind in datatypes.values()):
            return
        count = msg.width * msg.height
        if count == 0 or msg.point_step <= 0 or len(msg.data) < count * msg.point_step:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(count, msg.point_step)
        xyz = np.stack([
            raw[:, offsets[name]:offsets[name] + 4].copy().view(np.float32).reshape(-1)
            for name in ("x", "y", "z")
        ])
        frame = msg.header.frame_id or "base_link"
        if frame != "base_link":
            try:
                transform = self.buffer.lookup_transform(
                    "base_link", frame, msg.header.stamp, timeout=Duration(seconds=0.05)
                )
            except Exception:
                return  # No live transform: leave the scan stale rather than trust raw axes.
            xyz = rotation(transform) @ xyz + np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ])[:, None]
        near, far, ceiling = body_band(
            self.near, self.far, self.ceiling, self.lidar_height
        )
        keep = filter_xyz(xyz[0], xyz[1], xyz[2], near, far, self.ramp, ceiling)
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = "base_link"
        out.height = 1
        out.width = int(keep.sum())
        out.fields = msg.fields
        out.is_bigendian = False
        out.point_step = msg.point_step
        out.row_step = out.width * msg.point_step
        out.is_dense = msg.is_dense
        data = raw[keep].copy()
        for name, values in zip(("x", "y", "z"), xyz):
            offset = offsets[name]
            data[:, offset:offset + 4] = (
                values[keep].astype(np.float32).view(np.uint8).reshape(-1, 4)
            )
        out.data = data.tobytes()
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = CloudHeightFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
