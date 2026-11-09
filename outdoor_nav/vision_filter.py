"""Post-detection filter: exact-frame original geometry -> stable camera costmap."""
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
from std_msgs.msg import String, Header
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from shapely.geometry import Polygon
from semantic_direct_navigation.geometric_perception import observed_ground_region
from semantic_direct_navigation.costmap import build_costmap
from semantic_direct_navigation.core import grid_regions
from .configuration import load_runtime
from .core.camera_stability import StaticObstacles


def xyz(msg):
    values = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    return np.column_stack([values[n] for n in ("x", "y", "z")]).astype(float)


class VisionFilter(Node):
    def __init__(self, runtime):
        super().__init__("outdoor_visual_post_filter")
        self.p, self.topics = runtime["policy"], runtime["topic_remap"]
        self.filter = StaticObstacles(*(self.p[k] for k in ("camera_static_cell_m", "camera_static_min_hits", "camera_static_dwell_s", "camera_static_gap_s", "camera_static_drift_m")))
        self.frames = {}
        self.last_stamp = -math.inf
        self.last_now = -math.inf
        self.last_grid = None
        self.semantics = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(OccupancyGrid, self.topics["camera_costmap"], 1)
        self.health = self.create_publisher(LaserScan, self.topics["camera_scan"], 10)
        self.status = self.create_publisher(String, "/outdoor/vision/filter_status", 10)
        for kind, typ in (("obstacles", PointCloud2), ("ground", PointCloud2), ("status", String)):
            self.create_subscription(typ, self.topics["camera_"+kind], lambda msg, k=kind: self.receive(k, msg), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.topics["camera_semantics"], self.semantic, 5)

    def semantic(self, msg):
        self.semantics = msg

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
            from semantic_local_grid.ros_utils import matrix_from_tf
            from semantic_local_grid.geometry import transform as apply_transform
            body = apply_transform(obstacles, matrix_from_tf(transform))
            residual = body[:,2]-(body[:,:2] @ plane[:2]+plane[2])
            drops = residual < -.10
            positive_ids = np.flatnonzero(~drops)
            keep = self.filter.update(obstacles[positive_ids,:2].tolist(), stamp)
            selected = list(np.flatnonzero(drops))+[int(positive_ids[i]) for i in keep]
            stable = obstacles[selected, :2]
            pose = self.tf.lookup_transform("odom", "base_link", Time.from_msg(frame["obstacles"].header.stamp), timeout=Duration(seconds=.05)).transform.translation
            center = np.array([pose.x, pose.y])
            region = observed_ground_region(ground[:,:2], center)
            semantic_road, nonroad = Polygon(), Polygon()
            if self.p["pidnet_enabled"] and self.semantics is not None:
                sem = self.semantics
                sem_stamp = sem.header.stamp.sec+sem.header.stamp.nanosec/1e9
                if sem.header.frame_id == "odom" and 0 <= now-sem_stamp < self.p["sensor_timeout_s"]:
                    semantic_road, nonroad = grid_regions(np.asarray(sem.data).reshape(sem.info.height, sem.info.width),
                        sem.info.resolution, np.array([sem.info.origin.position.x, sem.info.origin.position.y]))
            _, costs, origin, resolution, _ = build_costmap(region, stable, np.empty((0,2)), center,
                                                          semantic_road=semantic_road, semantic_nonroad=nonroad)
            if self.get_clock().now().nanoseconds/1e9-stamp >= self.p["sensor_timeout_s"]:
                return
            output = OccupancyGrid()
            output.header = frame["obstacles"].header
            output.info.resolution = resolution
            output.info.height, output.info.width = costs.shape
            output.info.origin.position.x, output.info.origin.position.y = map(float, origin)
            output.info.origin.orientation.w = 1.
            output.data = np.minimum(costs, 100).astype(np.int8).ravel().tolist()
            self.pub.publish(output)
            self.last_grid = output
            alive = LaserScan()
            alive.header = output.header
            alive.range_min, alive.range_max, alive.angle_increment = .1, 6., .01
            alive.ranges = [math.nan]  # Health certificate only; not a costmap observation source.
            self.health.publish(alive)
            self.status.publish(String(data=json.dumps(dict(valid=True, stamp=stamp, raw=len(obstacles), stable=len(stable), preserved_drops=int(drops.sum())))))
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
