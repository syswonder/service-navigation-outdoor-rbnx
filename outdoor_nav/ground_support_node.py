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
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from semantic_local_grid.geometry import transform
from semantic_local_grid.ros_utils import matrix_from_tf

from .configuration import load_runtime
from .core.ground_support import (
    CELL, DEPTH_TOPIC, FRAME, GRID_TOPIC, HEIGHT_TOPIC, SIZE, SURVEY_GRID_TOPIC, SURVEY_HEIGHT_TOPIC,
    GroundGrid, SupportMask,
)


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
    def __init__(self, node, config, tf, *, with_heights=False, survey=False):
        self.node, self.config, self.tf = node, config, tf
        self.snapshot = None
        self.invalid_before = -math.inf
        self.heights = {}
        grid_topic = SURVEY_GRID_TOPIC if survey else GRID_TOPIC
        height_topic = SURVEY_HEIGHT_TOPIC if survey else HEIGHT_TOPIC
        node.create_subscription(OccupancyGrid, grid_topic, self.receive, 1)
        if with_heights:
            node.create_subscription(Image, height_topic, self.receive_height, 1)

    def receive_height(self, msg):
        if (msg.header.frame_id == FRAME and msg.width == SIZE and msg.height == SIZE
                and msg.encoding == "32FC1" and msg.step == SIZE*4 and len(msg.data) == SIZE*SIZE*4):
            stamp = seconds(msg.header.stamp)
            dtype = ">f4" if msg.is_bigendian else "<f4"
            values = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(SIZE, SIZE)
            self.heights = {**dict(list(self.heights.items())[-2:]), stamp: values}

    def elevation(self, sample):
        return self.heights.get(sample.stamp)

    def invalidate(self):
        self.snapshot = None
        self.heights = {}
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
        values = np.asarray(msg.data).reshape(SIZE, SIZE)
        cells = (values == 0) | (values == 1)
        stamp = seconds(msg.header.stamp)
        if stamp > self.invalid_before:
            if self.snapshot is not None and stamp <= self.snapshot[0].stamp:
                return
            sample = SupportMask(cells, (info.origin.position.x, info.origin.position.y),
                                 info.resolution, stamp, self.config["footprint_radius_m"],
                                 hazards=values == 100, depth_cells=values == 0)
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
        self.auto = self.p.get("ranger_descent_mode") == "auto"
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.grid = GroundGrid(self.p["ranger_step_limit_m"],
                               math.tan(math.radians(self.p["ranger_grade_limit_deg"])),
                               self.p["footprint_radius_m"])
        self.survey_grid = (GroundGrid(self.grid.step, self.grid.grade, self.grid.radius)
                            if self.auto else None)
        if self.auto:
            self.survey_pub = self.create_publisher(OccupancyGrid, SURVEY_GRID_TOPIC, 1)
            self.survey_height_pub = self.create_publisher(Image, SURVEY_HEIGHT_TOPIC, 1)
        self.pub = self.create_publisher(OccupancyGrid, GRID_TOPIC, 1)
        self.height_pub = self.create_publisher(Image, HEIGHT_TOPIC, 1)
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

    def reset_ground(self):
        for grid in (self.grid, self.survey_grid):
            if grid is not None:
                grid.height.fill(np.nan)
                grid.observed.fill(-np.inf)
                grid.depth_observed.fill(-np.inf)
                grid.rough.fill(False)
        self.lidar = None

    def on_geo(self, msg):
        try:
            geo = json.loads(msg.data)
            epoch = ((geo.get("fusion") or {}).get("recovery_id"),
                     (geo.get("navigation_sample") or {}).get("epoch"))
        except (ValueError, AttributeError):
            return
        if self.correction is not None and epoch != self.correction:
            # Discard the old support without seeding a different location.
            self.reset_ground()
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
            self.reset_ground()
        timeout = self.p["ranger_depth_timeout_s"]
        if not (0 < stamp and self.last_depth < stamp and 0 <= now-stamp < timeout):
            return
        self.last_depth = stamp
        lidar = self.lidar
        survey_support = None
        valid = True
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
            age = self.p["ranger_ground_cell_timeout_s"] if self.auto else 45.
            # LiDAR supplies close-range support outside the depth camera's
            # footprint. Beyond that strip auto mode requires observed depth.
            near_raw = (raw[np.linalg.norm(raw[:, :2] - xy, axis=1) <= self.grid.radius + .9]
                        if self.auto else raw)
            supported = self.grid.update(xy, ground, np.concatenate((depth, near_raw)),
                                         stamp, age, depth_count=len(depth),
                                         lidar_stamp=lidar_stamp if self.auto else None)
            if self.survey_grid is not None:
                survey_support = self.survey_grid.update(xy, ground, np.concatenate((depth, raw)),
                                                        stamp, depth_count=len(depth), lidar_stamp=lidar_stamp)
        except (TransformException, ValueError, TypeError, AssertionError):
            valid = False
            supported = np.zeros((SIZE, SIZE), dtype=bool)
            if self.survey_grid is not None:
                survey_support = supported
        self.publish_ground(msg, stamp, self.grid, supported, self.pub, self.height_pub, valid)
        if survey_support is not None:
            self.publish_ground(msg, stamp, self.survey_grid, survey_support,
                                self.survey_pub, self.survey_height_pub, valid)

    def publish_ground(self, msg, stamp, grid, supported, publisher, height_publisher, valid):
        if grid.origin is None:
            return
        output = OccupancyGrid()
        output.header = msg.header
        output.header.frame_id = FRAME
        output.info.resolution = CELL
        output.info.width = output.info.height = SIZE
        output.info.origin.position.x, output.info.origin.position.y = map(float, grid.origin)
        output.info.origin.orientation.w = 1.
        if self.auto:
            depth_fresh = stamp - grid.depth_observed <= self.p["ranger_ground_cell_timeout_s"]
            values = np.where(supported, np.where(depth_fresh, 0, 1), np.where(valid & grid.hazard, 100, -1))
        else:
            values = np.where(supported, 0, 100)
        output.data = values.astype(np.int8).ravel().tolist()
        heights = np.where(supported, grid.height, np.nan).astype("<f4")
        height_publisher.publish(Image(header=output.header, height=SIZE, width=SIZE,
                                       encoding="32FC1", step=SIZE*4, data=heights.tobytes()))
        publisher.publish(output)


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
