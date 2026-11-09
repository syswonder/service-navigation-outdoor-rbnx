"""costmap_fusion_node: fuse /semantic/local_grid + /obstacle/local_grid.

Applies the explicit three-state fusion rules and inflation, then publishes the
final costmap (/local_costmap_debug) plus separate debug grids so semantic road
boundary (100) and physical obstacle (101) stay distinguishable in the data.
"""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String
from .obstacle_geometry import fuse, inflate, FREE, NON_ROAD, OBSTACLE, UNKNOWN


class FusionNode(Node):
    def __init__(self, **kwargs):
        super().__init__('costmap_fusion_node', **kwargs)
        defaults = {
            'semantic_topic': '/semantic/local_grid',
            'obstacle_topic': '/obstacle/local_grid',
            'fused_topic': '/local_costmap_debug',
            'debug_semantic_topic': '/debug/semantic_grid',
            'debug_obstacle_topic': '/debug/obstacle_grid',
            'debug_fused_topic': '/debug/fused_grid',
            'resolution': 0.05, 'width': 160, 'height': 160,
            'inflation_radius': 0.3,
            'max_input_age': 1.0,
            'watchdog_timeout': 1.5,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.semantic = None
        self.obstacle = None
        self.sem_header = None
        self.obs_header = None
        self.last_sem_t = None
        self.last_obs_t = None
        self.fused_pub = self.create_publisher(OccupancyGrid, self.p['fused_topic'], 5)
        self.dbg_sem = self.create_publisher(OccupancyGrid, self.p['debug_semantic_topic'], 5)
        self.dbg_obs = self.create_publisher(OccupancyGrid, self.p['debug_obstacle_topic'], 5)
        self.dbg_fused = self.create_publisher(OccupancyGrid, self.p['debug_fused_topic'], 5)
        self.status_pub = self.create_publisher(String, '/fusion/status', 10)
        self.sem_sub = self.create_subscription(OccupancyGrid, self.p['semantic_topic'], self.on_semantic, 10)
        self.obs_sub = self.create_subscription(OccupancyGrid, self.p['obstacle_topic'], self.on_obstacle, 10)
        self.timer = self.create_timer(0.25, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Costmap fusion ready')

    def on_semantic(self, msg):
        self.semantic = self.to_grid(msg)
        self.sem_header = msg.header
        self.last_sem_t = time.monotonic()
        self.recompute()

    def on_obstacle(self, msg):
        self.obstacle = self.to_grid(msg)
        self.obs_header = msg.header
        self.last_obs_t = time.monotonic()
        self.recompute()

    @staticmethod
    def to_grid(msg):
        return np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)

    def to_msg(self, grid, header):
        g = np.asarray(grid, dtype=np.int8)
        m = OccupancyGrid()
        m.header = header
        m.info.map_load_time = header.stamp
        m.info.resolution = self.p['resolution']
        m.info.width = self.p['width']
        m.info.height = self.p['height']
        m.info.origin.position.x = 0.0
        m.info.origin.position.y = 0.0
        m.data = g.reshape(-1).tolist()
        return m

    def recompute(self):
        if self.semantic is None or self.obstacle is None:
            return
        if self.semantic.shape != self.obstacle.shape:
            self.get_logger().warn('grid shape mismatch; waiting for aligned grids')
            return
        fused = fuse(self.semantic, self.obstacle)
        inflated = inflate(fused, self.p['resolution'], self.p['inflation_radius'])
        # debug passthroughs + raw fused (100 vs 101 distinguish semantic/obstacle)
        self.dbg_sem.publish(self.to_msg(self.semantic, self.sem_header))
        self.dbg_obs.publish(self.to_msg(self.obstacle, self.obs_header))
        self.dbg_fused.publish(self.to_msg(fused, self.sem_header))
        self.fused_pub.publish(self.to_msg(inflated, self.sem_header))
        self.status_pub.publish(String(data=json.dumps({
            'ok': True,
            'free_cells': int((inflated == FREE).sum()),
            'inflated_cells': int(((inflated > 0) & (inflated < NON_ROAD)).sum()),
            'nonroad_cells': int((inflated == NON_ROAD).sum()),
            'obstacle_cells': int((inflated == OBSTACLE).sum()),
            'unknown_cells': int((inflated == UNKNOWN).sum()),
            'inflation_radius': self.p['inflation_radius'],
        })))

    def watchdog(self):
        """If either input is stale, fail closed: publish an all-UNKNOWN fused grid."""
        now = time.monotonic()
        stale = []
        if self.last_sem_t is None or now - self.last_sem_t > self.p['watchdog_timeout']:
            stale.append('semantic')
        if self.last_obs_t is None or now - self.last_obs_t > self.p['watchdog_timeout']:
            stale.append('obstacle')
        if stale:
            g = np.full((self.p['height'], self.p['width']), UNKNOWN, np.int8)
            if self.sem_header is not None:
                self.fused_pub.publish(self.to_msg(g, self.sem_header))
            self.status_pub.publish(String(data=json.dumps(
                {'ok': False, 'reason': 'stale input: ' + ','.join(stale)})))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
