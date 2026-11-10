"""Filter a point cloud by a distance-graded minimum height before scan projection."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from .core.height_filter import filter_xyz


class CloudHeightFilter(Node):
    def __init__(self):
        super().__init__("outdoor_cloud_height_filter")
        self.near = float(self.declare_parameter("near_m", 0.2).value)
        self.far = float(self.declare_parameter("far_m", 0.5).value)
        self.ramp = float(self.declare_parameter("ramp_m", 8.0).value)
        self.ceiling = float(self.declare_parameter("max_m", 1.5).value)
        if not (0 <= self.near <= self.far and self.ramp > 0 and self.ceiling > self.near):
            raise ValueError("Invalid graded height parameters")
        self.publisher = self.create_publisher(PointCloud2, "/outdoor/cloud_for_scan", 10)
        self.create_subscription(PointCloud2, "cloud_in", self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        offsets = {}
        for field in msg.fields:
            if field.name in ("x", "y", "z"):
                offsets[field.name] = field.offset
        if len(offsets) != 3 or msg.is_bigendian:
            return
        names = sorted(offsets)
        if any(msg.fields[[f.name for f in msg.fields].index(n)].datatype != PointField.FLOAT32 for n in names):
            return
        count = msg.width * msg.height
        if count == 0 or msg.point_step <= 0 or len(msg.data) < count * msg.point_step:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(count, msg.point_step)
        x, y, z = (
            raw[:, offsets[name]:offsets[name] + 4].copy().view(np.float32).reshape(-1)
            for name in names
        )
        keep = filter_xyz(x, y, z, self.near, self.far, self.ramp, self.ceiling)
        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(keep.sum())
        out.fields = msg.fields
        out.is_bigendian = False
        out.point_step = msg.point_step
        out.row_step = out.width * msg.point_step
        out.data = raw[keep].tobytes()
        out.is_dense = msg.is_dense
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
