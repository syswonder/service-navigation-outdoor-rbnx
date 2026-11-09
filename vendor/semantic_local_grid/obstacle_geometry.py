"""Pure LiDAR-obstacle geometry and semantic/obstacle fusion. No ROS imports.

This module owns the obstacle pipeline's pure computation so it can be unit-tested
without rclpy/tf. The occupancy/cost scheme is int8 (OccupancyGrid) so RViz Map
renders it directly, and semantic non-road vs physical obstacle stay distinguishable:

- -1    UNKNOWN
- 0     FREE (road, no obstacle)
- 1..99 INFLATED (distance-based cost ring; higher = closer to occupied)
- 100   NON_ROAD  (semantic road boundary)
- 101   OBSTACLE  (physical LiDAR obstacle; distinct from NON_ROAD for debugging)

Input-grid conventions (kept separate so the two upstream nodes stay independent):

- semantic grid: -1 UNKNOWN, 0 ROAD, 100 NON_ROAD (existing projection node)
- obstacle grid: -1 UNKNOWN, 100 OBSTACLE (obstacle node; LiDAR only marks obstacles,
  never clears, never judges road)
"""
import numpy as np
import cv2

UNKNOWN = -1
FREE = 0
NON_ROAD = 100
OBSTACLE = 101

SEM_UNKNOWN = -1
SEM_ROAD = 0
SEM_NON_ROAD = 100

OBS_UNKNOWN = -1
OBS_OBSTACLE = 100

INFLATED_MIN = 1
INFLATED_MAX = 99


def filter_height(points, min_h, max_h):
    """Keep points whose planning-frame z lies in [min_h, max_h]."""
    points = np.asarray(points, dtype=np.float32)
    if len(points) == 0:
        return points
    z = points[:, 2]
    return points[(z >= min_h) & (z <= max_h)]


def filter_range(points, range_min, range_max, robot_xy=(0.0, 0.0)):
    """Keep points whose xy distance from robot_xy lies in [range_min, range_max]."""
    points = np.asarray(points, dtype=np.float32)
    if len(points) == 0:
        return points
    r = np.hypot(points[:, 0] - robot_xy[0], points[:, 1] - robot_xy[1])
    return points[(r >= range_min) & (r <= range_max)]


def voxel_downsample(points, voxel_size):
    """Collapse points into one centroid per voxel (3-D). Returns centroid array."""
    points = np.asarray(points, dtype=np.float32)
    if voxel_size <= 0 or len(points) == 0:
        return points
    q = np.floor(points / voxel_size).astype(np.int64)
    _, inverse, counts = np.unique(q, axis=0, return_inverse=True, return_counts=True)
    out = np.zeros((len(counts), 3), dtype=np.float32)
    np.add.at(out, inverse, points)
    out /= counts[:, None]
    return out


def fuse(semantic, obstacle):
    """Explicit three-state fusion. Physical obstacle wins; semantic never clears it.

    Rules (per spec):
      obstacle LETHAL            -> OBSTACLE (101)
      else semantic NON_ROAD     -> NON_ROAD (100)
      else semantic ROAD         -> FREE (0)
      else                       -> UNKNOWN (-1)
    """
    semantic = np.asarray(semantic, dtype=np.int16)
    obstacle = np.asarray(obstacle, dtype=np.int16)
    if semantic.shape != obstacle.shape:
        raise ValueError('semantic/obstacle grid shapes differ')
    fused = np.full(semantic.shape, UNKNOWN, dtype=np.int16)
    fused[obstacle == OBS_OBSTACLE] = OBSTACLE
    fused[(obstacle != OBS_OBSTACLE) & (semantic == SEM_NON_ROAD)] = NON_ROAD
    fused[(obstacle != OBS_OBSTACLE) & (semantic == SEM_ROAD)] = FREE
    return fused


def inflate(grid, resolution, inflation_radius):
    """Inflate around NON_ROAD and OBSTACLE cells with a linear cost ramp (1..99).

    Never lowers an existing occupied cost and never creates evidence outside the
    radius. Cells already FREE or UNKNOWN inside the ring get a distance-based cost.
    """
    grid = np.asarray(grid, dtype=np.int16)
    if inflation_radius <= 0:
        return grid.copy()
    occupied = (grid == NON_ROAD) | (grid == OBSTACLE)
    if not occupied.any():
        return grid.copy()
    # distanceTransform: distance (pixels) to nearest zero => nearest occupied cell.
    dist_px = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 5)
    dist_m = dist_px.astype(np.float32) * resolution
    in_ring = (dist_m > 0) & (dist_m <= inflation_radius)
    out = grid.copy()
    if in_ring.any():
        cost = np.rint(INFLATED_MAX * (1.0 - dist_m / inflation_radius)).astype(np.int16)
        cost = np.clip(cost, INFLATED_MIN, INFLATED_MAX)
        out[in_ring] = cost[in_ring]
    return out
