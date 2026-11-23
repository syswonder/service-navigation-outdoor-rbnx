"""ROS transport for synchronized, capture-stamped Ranger support grids."""

import json
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from semantic_local_grid.geometry import transform
from semantic_local_grid.ros_utils import matrix_from_tf

from .configuration import load_runtime
from .core.ground_support import CELL, DEPTH_TOPIC, FRAME, GRID_TOPIC, SIZE, GroundGrid, SupportMask


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def latest_base(tf, node, timeout):
    pose = tf.lookup_transform(FRAME, "base_link", Time())
    age = node.get_clock().now().nanoseconds / 1e9 - seconds(pose.header.stamp)
    if not 0 <= age <= timeout:
        raise ValueError("Stale ground support pose")
    return pose


def xy_yaw(pose):
    p, q = pose.transform.translation, pose.transform.rotation
    return (p.x, p.y), math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class SupportFeed:
    def __init__(self, node, config, tf):
        self.node, self.config, self.tf = node, config, tf
        self.snapshot = None
        self.invalid_before = -math.inf
        node.create_subscription(OccupancyGrid, GRID_TOPIC, self.receive, 1)

    def invalidate(self):
        self.snapshot = None
        self.invalid_before = self.node.get_clock().now().nanoseconds / 1e9

    def receive(self, msg):
        info = msg.info
        q = info.origin.orientation
        if ((q.x, q.y, q.z, q.w) != (0., 0., 0., 1.)
                or msg.header.frame_id != FRAME or info.width != SIZE or info.height != SIZE
                or not math.isclose(info.resolution, CELL, abs_tol=1e-6)
                or len(msg.data) != SIZE*SIZE
                or not math.isfinite(info.origin.position.x) or not math.isfinite(info.origin.position.y)):
            self.snapshot = None
            return
        cells = np.asarray(msg.data).reshape(SIZE, SIZE) == 0
        stamp = seconds(msg.header.stamp)
        if stamp > self.invalid_before:
            sample = SupportMask(cells, (info.origin.position.x, info.origin.position.y),
                                 info.resolution, stamp, self.config["footprint_radius_m"])
            self.snapshot = sample, time.monotonic()

    def fresh(self):
        snapshot = self.snapshot
        if snapshot is None:
            return None
        sample, received = snapshot
        timeout = self.config["ranger_depth_timeout_s"]
        if (sample.stamp > self.invalid_before and time.monotonic() - received <= timeout
                and 0 <= self.node.get_clock().now().nanoseconds / 1e9 - sample.stamp <= timeout):
            return sample
        return None


class GroundSupport(Node):
    def __init__(self, runtime):
        super().__init__("ranger_ground_support")
        self.p = runtime["policy"]
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.grid = GroundGrid(self.p["ranger_step_limit_m"],
                               math.tan(math.radians(self.p["ranger_grade_limit_deg"])),
                               self.p["footprint_radius_m"])
        self.pub = self.create_publisher(OccupancyGrid, GRID_TOPIC, 1)
        self.lidar = None
        self.correction = None
        self.last_depth = -math.inf
        self.last_clock = -math.inf
        self.create_subscription(PointCloud2, runtime["topic_remap"]["cloud"], self.on_lidar,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, DEPTH_TOPIC, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(String, "/outdoor/localization", self.on_geo, 10)

    def on_lidar(self, msg):
        self.lidar = msg

    def on_geo(self, msg):
        try:
            geo = json.loads(msg.data)
            epoch = ((geo.get("fusion") or {}).get("recovery_id"),
                     (geo.get("navigation_sample") or {}).get("epoch"))
        except (ValueError, AttributeError):
            return
        if self.correction is not None and epoch != self.correction:
            # Discard the old support without seeding a different location.
            self.grid.height.fill(np.nan)
            self.grid.observed.fill(-np.inf)
            self.grid.rough.fill(False)
            self.lidar = None
        self.correction = epoch

    def points(self, msg):
        xyz = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.column_stack([xyz[name] for name in ("x", "y", "z")])
        if msg.header.frame_id != FRAME:
            tf = self.tf.lookup_transform(FRAME, msg.header.frame_id, Time.from_msg(msg.header.stamp))
            xyz = transform(xyz, matrix_from_tf(tf))
        return xyz

    def on_depth(self, msg):
        stamp = seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds / 1e9
        cleared = now < self.last_clock
        self.last_clock = now
        if cleared:
            self.last_depth = -math.inf
            self.grid.height.fill(np.nan)
            self.grid.observed.fill(-np.inf)
            self.grid.rough.fill(False)
            self.lidar = None
        timeout = self.p["ranger_depth_timeout_s"]
        if not (0 < stamp and self.last_depth < stamp and 0 <= now-stamp < timeout):
            return
        self.last_depth = stamp
        lidar = self.lidar
        try:
            pose = self.tf.lookup_transform(FRAME, "base_link", Time.from_msg(msg.header.stamp))
            xy, _ = xy_yaw(pose)
            ground = pose.transform.translation.z + self.p["camera_expected_ground_z_m"]
            lidar_stamp = seconds(lidar.header.stamp) if lidar is not None else -math.inf
            if (lidar is None or not 0 <= now-lidar_stamp < self.p["sensor_timeout_s"]
                    or abs(stamp-lidar_stamp) > .25 or msg.header.frame_id != FRAME or not msg.width):
                raise ValueError("Missing current depth or raw lidar")
            depth, raw = self.points(msg), self.points(lidar)
            if not len(depth) or not len(raw):
                raise ValueError("Empty depth or raw lidar")
            supported = self.grid.update(xy, ground, np.concatenate((depth, raw)), stamp)
        except (TransformException, ValueError, TypeError, AssertionError):
            supported = np.zeros((SIZE, SIZE), dtype=bool)
        if self.grid.origin is None:
            return
        output = OccupancyGrid()
        output.header = msg.header
        output.header.frame_id = FRAME
        output.info.resolution = CELL
        output.info.width = output.info.height = SIZE
        output.info.origin.position.x, output.info.origin.position.y = map(float, self.grid.origin)
        output.info.origin.orientation.w = 1.
        output.data = np.where(supported, 0, 100).astype(np.int8).ravel().tolist()
        self.pub.publish(output)


def main():
    path = sys.argv[1]
    rclpy.init(args=sys.argv[2:])
    node = GroundSupport(load_runtime(path))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
