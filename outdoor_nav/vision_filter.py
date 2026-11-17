"""Publish camera costs from stable geometry, semantic projection, or both."""
import json
import math
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vendor"))
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from shapely.geometry import Polygon
from semantic_direct_navigation.geometric_perception import observed_ground_region
from semantic_direct_navigation.costmap import build_costmap
from semantic_direct_navigation.core import grid_regions
from semantic_local_grid.geometry import transform as apply_transform
from semantic_local_grid.ros_utils import matrix_from_tf
from .configuration import load_runtime
from .core.camera_grid import allow_blind_zone, semantic_costs
from .core.camera_stability import StaticObstacles
from .core.terrain_geometry import forward_mask, suppress_stair_points
from .terrain_inputs import TerrainInputs


def xyz(msg):
    values = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    return np.column_stack([values[n] for n in ("x", "y", "z")]).astype(float)


class VisionFilter(Node):
    def __init__(self, runtime):
        super().__init__("outdoor_visual_post_filter")
        self.segmentation_enabled = runtime["semantic_segmentation"]["enabled"]
        self.p, self.topics = runtime["policy"], runtime["topic_remap"]
        self.geometry_enabled = self.p["camera_geometry_enabled"]
        self.filter = StaticObstacles(*(self.p[k] for k in ("camera_static_cell_m", "camera_static_min_hits", "camera_static_dwell_s", "camera_static_gap_s", "camera_static_drift_m")))
        self.frames = {}
        self.last_stamp = -math.inf
        self.last_now = -math.inf
        self.last_grid = None
        self.semantics = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.terrain = runtime.get("terrain", {})
        self.terrain_inputs = TerrainInputs(self, runtime) if self.terrain.get("enabled") else None
        self.pub = self.create_publisher(OccupancyGrid, self.topics["camera_costmap"], 1)
        self.health = self.create_publisher(LaserScan, self.topics["camera_scan"], 10)
        self.status = self.create_publisher(String, "/outdoor/vision/filter_status", 10)
        if self.geometry_enabled:
            for kind, typ in (("obstacles", PointCloud2), ("ground", PointCloud2), ("status", String)):
                self.create_subscription(typ, self.topics["camera_"+kind], lambda msg, k=kind: self.receive(k, msg), qos_profile_sensor_data)
        if self.segmentation_enabled:
            self.create_subscription(OccupancyGrid, self.topics["camera_semantics"], self.semantic, 5)

    def semantic(self, msg):
        if self.geometry_enabled:
            self.semantics = msg
            return
        try:
            now = self.get_clock().now().nanoseconds / 1e9
            stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
            if now < self.last_now:
                self.last_stamp = -math.inf
            self.last_now = now
            if stamp <= self.last_stamp or stamp <= 0 or not 0 <= now - stamp < self.p["sensor_timeout_s"]:
                return
            self.last_stamp = stamp
            info = msg.info
            origin = (info.origin.position.x, info.origin.position.y)
            q = info.origin.orientation
            if (msg.header.frame_id != "odom"
                    or not all(math.isfinite(v) for v in (*origin, info.resolution))
                    or info.resolution <= 0 or (q.x, q.y, q.z, q.w) != (0, 0, 0, 1)):
                raise ValueError("Invalid semantic projection geometry")
            costs = semantic_costs(np.asarray(msg.data).reshape(info.height, info.width))
            if not np.any(costs >= 0):
                raise ValueError("No classified ground in fresh semantic projection")
            transform = self.tf.lookup_transform("base_link", "odom", Time.from_msg(msg.header.stamp),
                                                     timeout=Duration(seconds=.05))
            self.publish_grid(msg.header, costs, origin, info.resolution, transform, mode="semantic")
        except Exception as exc:
            self.publish_invalid(str(exc))

    def publish_grid(self, header, costs, origin, resolution, transform, **status):
        stamp = Time.from_msg(header.stamp).nanoseconds / 1e9
        if transform is not None:
            rr, cc = np.indices(costs.shape)
            world = np.column_stack((origin[0]+(cc.ravel()+.5)*resolution,
                                     origin[1]+(rr.ravel()+.5)*resolution,
                                     np.zeros(costs.size)))
            body = apply_transform(world, matrix_from_tf(transform))
            costs = costs.copy()
            costs.ravel()[~forward_mask(body, self.p["camera_max_range_m"])] = -1
        if self.p["camera_blind_zone_m"]:
            costs = allow_blind_zone(costs, origin, resolution,
                                    matrix_from_tf(transform), self.p["camera_blind_zone_m"])
        if not 0 <= self.get_clock().now().nanoseconds / 1e9 - stamp < self.p["sensor_timeout_s"]:
            return
        output = OccupancyGrid()
        output.header = header
        output.info.resolution = resolution
        output.info.height, output.info.width = costs.shape
        output.info.origin.position.x, output.info.origin.position.y = map(float, origin)
        output.info.origin.orientation.w = 1.0
        output.data = np.minimum(costs, 100).astype(np.int8).ravel().tolist()
        self.pub.publish(output)
        self.last_grid = output
        alive = LaserScan()
        alive.header = header
        alive.range_min, alive.range_max, alive.angle_increment = .1, 6., .01
        alive.ranges = [math.nan]  # Health certificate only; not a costmap observation source.
        self.health.publish(alive)
        self.status.publish(String(data=json.dumps(dict(valid=True, stamp=stamp, **status))))

    def receive(self, kind, msg):
        try:
            status = json.loads(msg.data) if kind == "status" else None
            stamp = float(status["stamp"]) if status is not None else msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            now = self.get_clock().now().nanoseconds/1e9
            if not math.isfinite(stamp) or not 0 <= now-stamp < self.p["sensor_timeout_s"]:
                return
            if now < self.last_now:
                self.frames.clear()
                self.filter = StaticObstacles(*(self.p[k] for k in ("camera_static_cell_m", "camera_static_min_hits", "camera_static_dwell_s", "camera_static_gap_s", "camera_static_drift_m")))
                self.last_stamp = -math.inf
            self.last_now = now
            if stamp <= self.last_stamp:
                return
            self.frames = {t: f for t, f in self.frames.items() if now-t < self.p["sensor_timeout_s"]}
            self.frames.setdefault(stamp, {})[kind] = msg
            while len(self.frames) > 12:
                self.frames.pop(min(self.frames))
            frame = self.frames[stamp]
            if status is not None and not status.get("valid"):
                self.last_stamp = stamp
                self.filter.tracks = []
                self.publish_invalid(msg.data)
                return
            if not all(k in frame for k in ("obstacles", "ground", "status")):
                return
            certificate = json.loads(frame["status"].data)
            if not certificate.get("valid") or any(frame[k].header.frame_id != "odom" for k in ("obstacles", "ground")):
                return
            obstacles, ground = xyz(frame["obstacles"]), xyz(frame["ground"])
            plane = np.asarray(certificate["plane_abc"], float)
            # The original depth certificate's plane is in base_link; preserve drop labels
            # using the source-frame transform rather than applying it in odom.
            transform = self.tf.lookup_transform("base_link", "odom", Time.from_msg(frame["obstacles"].header.stamp), timeout=Duration(seconds=.05))
            body = apply_transform(obstacles, matrix_from_tf(transform))
            valid = forward_mask(body, self.p["camera_max_range_m"])
            obstacles, body = obstacles[valid], body[valid]
            ground_body = apply_transform(ground, matrix_from_tf(transform))
            ground = ground[forward_mask(ground_body, self.p["camera_max_range_m"])]
            residual = body[:,2]-(body[:,:2] @ plane[:2]+plane[2])
            drops = residual < -.10
            accepted = np.zeros(len(obstacles), bool)
            if self.terrain.get("slope_filter_enabled") and self.terrain.get("camera_slope_enabled", True):
                # Negative obstacles remain immediate even inside the slope envelope.
                threshold = self.terrain["height_margin_m"] + np.linalg.norm(body[:,:2], axis=1)*math.tan(math.radians(self.terrain["slope_angle_deg"]))
                accepted |= (body[:,2]-self.terrain["ground_z_m"] <= threshold) & ~drops & (body[:,2] >= self.terrain["ground_z_m"])
            if self.terrain_inputs:
                obs, state = self.terrain_inputs.suppression_context()
                accepted |= suppress_stair_points(obstacles, obs, state, "visual_obstacles", self.terrain) & ~drops
            ground = np.vstack((ground, obstacles[accepted]))
            obstacles, body, drops = obstacles[~accepted], body[~accepted], drops[~accepted]
            positive_ids = np.flatnonzero(~drops)
            keep = self.filter.update(obstacles[positive_ids,:2].tolist(), stamp)
            selected = list(np.flatnonzero(drops))+[int(positive_ids[i]) for i in keep]
            stable = obstacles[selected, :2]
            pose = self.tf.lookup_transform("odom", "base_link", Time.from_msg(frame["obstacles"].header.stamp), timeout=Duration(seconds=.05)).transform.translation
            center = np.array([pose.x, pose.y])
            region = observed_ground_region(ground[:,:2], center)
            semantic_road, nonroad = Polygon(), Polygon()
            if self.segmentation_enabled and self.semantics is not None:
                sem = self.semantics
                sem_stamp = sem.header.stamp.sec+sem.header.stamp.nanosec/1e9
                if sem.header.frame_id == "odom" and 0 <= now-sem_stamp < self.p["sensor_timeout_s"]:
                    semantic_road, nonroad = grid_regions(np.asarray(sem.data).reshape(sem.info.height, sem.info.width),
                        sem.info.resolution, np.array([sem.info.origin.position.x, sem.info.origin.position.y]))
            _, costs, origin, resolution, _ = build_costmap(region, stable, np.empty((0,2)), center,
                                                          semantic_road=semantic_road, semantic_nonroad=nonroad)
            self.publish_grid(frame["obstacles"].header, costs, origin, resolution, transform,
                              mode="geometry", raw=len(obstacles), stable=len(stable),
                              preserved_drops=int(drops.sum()))
            self.last_stamp = stamp
            self.frames = {t: f for t, f in self.frames.items() if t > stamp}
        except Exception as exc:
            self.publish_invalid(str(exc))

    def publish_invalid(self, reason):
        if self.last_grid is not None:
            import copy
            blank = copy.deepcopy(self.last_grid)
            blank.header.stamp = self.get_clock().now().to_msg()
            blank.data = [-1]*len(blank.data)
            self.pub.publish(blank)
        self.health.publish(LaserScan())
        self.status.publish(String(data=json.dumps({"valid": False, "reason": reason})))


def main():
    runtime = load_runtime(sys.argv[1])
    rclpy.init(args=sys.argv[2:])
    node = VisionFilter(runtime)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
