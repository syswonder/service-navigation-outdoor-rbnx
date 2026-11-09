"""Pure geometry and trajectory rollouts, independent of ROS and costmap."""
from dataclasses import dataclass, field
import math
import time
import numpy as np
from scipy.spatial import cKDTree
from scipy import ndimage
from shapely.geometry import Polygon, LineString, Point, box
from shapely.ops import unary_union
import shapely


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def integrate(pose, v, w, dt):
    x, y, yaw = pose
    if abs(w) < 1e-8:
        return np.array([x + v * dt * math.cos(yaw), y + v * dt * math.sin(yaw), yaw])
    end = yaw + w * dt
    return np.array([
        x + v / w * (math.sin(end) - math.sin(yaw)), y - v / w * (math.cos(end) - math.cos(yaw)),
        wrap(end)
    ])


def grid_regions(grid, resolution, origin, *, include_nonroad=True):
    """Union observed cell runs, preserving exact cell edges and unknown holes."""
    grid = np.asarray(grid)
    regions = []
    for value in (0, 100):
        if value == 100 and not include_nonroad:
            regions.append(Polygon())
            continue
        mask = grid == value
        if not mask.any():
            regions.append(Polygon())
            continue
        # Extract ONLY exposed cell edges. Interior edges cancel exactly;
        # polygonization avoids overlaying thousands of cell/run rectangles.
        # Unit edges keep all junctions noded, including diagonal contacts.
        hr, hc = np.nonzero(np.diff(np.pad(mask.astype(np.int8), ((1, 1), (0, 0))), axis=0))
        vr, vc = np.nonzero(np.diff(np.pad(mask.astype(np.int8), ((0, 0), (1, 1))), axis=1))
        horizontal = np.stack((np.column_stack((hc, hr)), np.column_stack((hc+1, hr))), axis=1)
        vertical = np.stack((np.column_stack((vc, vr)), np.column_stack((vc, vr+1))), axis=1)
        edges = np.concatenate((horizontal, vertical)).astype(float)
        edges = np.asarray(origin) + edges * resolution
        faces = shapely.get_parts(shapely.polygonize(shapely.linestrings(edges)))
        # Polygonize also yields faces inside holes. Keep only faces whose
        # interior sample belongs to this label; unknown/nonroad stay holes.
        xy = shapely.get_coordinates(shapely.point_on_surface(faces))
        cells = np.floor((xy-np.asarray(origin))/resolution).astype(int)
        keep = mask[cells[:, 1], cells[:, 0]]
        geometry = shapely.multipolygons(faces[keep])
        regions.append(geometry.simplify(0, preserve_topology=True))
    return regions


def repair_ground_holes(grid,
                        resolution,
                        origin,
                        ground_xy,
                        obstacle_xy,
                        max_area=.09,
                        max_span=.35,
                        support_distance=.075):
    """Fill small enclosed NONROAD patches only with measured flat-ground evidence."""
    grid = np.asarray(grid)
    out = grid.copy()
    ground_xy = np.asarray(ground_xy, float).reshape(-1, 2)
    obstacles = np.asarray(obstacle_xy, float).reshape(-1, 2)
    ground_xy = ground_xy[np.isfinite(ground_xy).all(axis=1)]
    obstacles = obstacles[np.isfinite(obstacles).all(axis=1)]
    stats = {'repaired_cells': 0, 'repaired_components': 0}
    if not len(ground_xy) or resolution <= 0: return out, stats
    labels, _ = ndimage.label(grid == 100, structure=np.ones((3, 3)))
    ground_tree = cKDTree(ground_xy)
    # Nearby road points must not stand in for a measurement INSIDE a raised patch.
    cells = np.floor((ground_xy - np.asarray(origin)) / resolution).astype(int)
    keep = (cells[:, 0] >= 0) & (cells[:, 0] < grid.shape[1]) & (cells[:, 1] >= 0) & (
        cells[:, 1] < grid.shape[0])
    measured_ground = np.zeros(grid.shape, dtype=bool)
    measured_ground[cells[keep, 1], cells[keep, 0]] = True
    obstacle_tree = cKDTree(obstacles) if len(obstacles) else None
    for number, bounds in enumerate(ndimage.find_objects(labels), 1):
        if bounds is None: continue
        rr, cc = bounds
        if rr.start == 0 or cc.start == 0 or rr.stop == grid.shape[0] or cc.stop == grid.shape[1]:
            continue
        component = labels[rr.start - 1:rr.stop + 1, cc.start - 1:cc.stop + 1] == number
        n = int(component.sum())
        if n * resolution**2 > max_area + 1e-9 or max(
                rr.stop - rr.start, cc.stop - cc.start) * resolution > max_span + 1e-9:
            continue
        ring = ndimage.binary_dilation(component, structure=np.ones((3, 3))) & ~component
        local = grid[rr.start - 1:rr.stop + 1, cc.start - 1:cc.stop + 1]
        if not np.all(local[ring] == 0): continue
        rows, cols = np.nonzero(component)
        rows += rr.start - 1
        cols += cc.start - 1
        if not np.all(measured_ground[rows, cols]): continue
        centers = np.column_stack(
            (origin[0] + (cols + .5) * resolution, origin[1] + (rows + .5) * resolution))
        if np.any(ground_tree.query(centers)[0] > support_distance): continue
        if obstacle_tree is not None and np.any(
                obstacle_tree.query(centers)[0] <= resolution * np.sqrt(2) + .03):
            continue
        out[rows, cols] = 0
        stats['repaired_cells'] += n
        stats['repaired_components'] += 1
    return out, stats


def nearfield_corridor(pose, forward_m, safety_radius):
    """Operator-confirmed local traversability prior at the supplied current pose.

    This is an explicit assumption, never sensor evidence. The extra radius
    is the planner's swept-envelope padding, not permission for lateral travel.
    """
    pose = np.asarray(pose, float)
    if pose.shape != (3, ) or not np.isfinite(pose).all():
        raise ValueError('Invalid nearfield pose')
    if not np.isfinite(forward_m) or not 0 <= forward_m <= 1.0:
        raise ValueError('Nearfield corridor exceeds confirmed 1 m')
    if not np.isfinite(safety_radius) or safety_radius <= 0:
        raise ValueError('Invalid nearfield radius')
    end = pose[:2] + forward_m * np.array([np.cos(pose[2]), np.sin(pose[2])])
    shape = LineString([pose[:2], end]) if forward_m else Point(pose[:2])
    return shape.buffer(safety_radius + .025, quad_segs=32)


def nearfield_visual_policy(pose,
                            visual_xy,
                            *,
                            enabled=False,
                            forward_m=1.,
                            half_length=.42,
                            half_width=.3):
    """Retain all positive visual obstacle evidence, including the blind zone.

    The legacy arguments remain accepted for old launch files. A traversability
    prior may fill a camera blind area, but cannot erase measured obstacles.
    Missing LiDAR returns are not evidence that a visual obstacle is absent.
    """
    points = np.asarray(visual_xy, dtype=float).reshape(-1, 2)
    if not enabled: return points, 0
    pose = np.asarray(pose, dtype=float)
    if (pose.shape != (3, ) or not np.isfinite(pose).all()
            or not np.isfinite([forward_m, half_length, half_width]).all()
            or not 0 <= forward_m <= 1. or half_length <= 0 or half_width <= 0):
        raise ValueError('Invalid operator nearfield bounds')
    return points, 0


def obstacle_proximity(pose, points):
    """Nearest raw obstacle, with body coordinates for field diagnosis."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(points): return None
    distances = np.linalg.norm(points - np.asarray(pose)[:2], axis=1)
    i = int(np.argmin(distances))
    c, s = np.cos(pose[2]), np.sin(pose[2])
    body = (points[i] - pose[:2]) @ np.array([[c, -s], [s, c]])
    return dict(distance_m=float(distances[i]), forward_m=float(body[0]), left_m=float(body[1]))


class RoadMemory:
    """Bounded geometry memory. Current observations overwrite old semantics."""

    def __init__(self, ttl=10.0, max_batches=30, window=20.0):
        self.ttl, self.max_batches, self.window = ttl, max_batches, window
        self.batches = []
        self.last_stamp = -np.inf
        self.observation_stamps = []
        self._geometry_cache = None

    def update(self, road, nonroad, stamp, center):
        if stamp <= self.last_stamp:
            if stamp < self.last_stamp:
                self.batches.clear()
                self.observation_stamps.clear()
            else:
                return
        self.last_stamp = stamp
        self.observation_stamps = (self.observation_stamps + [stamp])[-self.max_batches:]
        oldest = self.observation_stamps[0]
        # ROS OccupancyGrid resolution is float32, whereas origins are float64.
        # Canonical micrometre precision prevents repeated overlays from making
        # sub-micrometre slivers. The planner already adds >1 um safety padding.
        road = shapely.set_precision(road, 1e-6)
        nonroad = shapely.set_precision(nonroad, 1e-6)
        self._geometry_cache = None
        observed = road.union(nonroad)
        self.batches = [(t, g.difference(observed)) for t, g in self.batches
                        if 0 <= stamp - t <= self.ttl and t >= oldest and not g.is_empty]
        self.batches.append((stamp, road))
        clip = box(center[0] - self.window / 2, center[1] - self.window / 2,
                   center[0] + self.window / 2, center[1] + self.window / 2)
        self.batches = [(t, g.intersection(clip)) for t, g in self.batches[-self.max_batches:]]
        self.batches = [(t, g) for t, g in self.batches if not g.is_empty]

    def geometry(self, now):
        live = [(t, g) for t, g in self.batches if 0 <= now - t <= self.ttl]
        key = tuple(t for t, _ in live)
        if self._geometry_cache is None or self._geometry_cache[0] != key:
            # Current-frame mode already supplies a unioned geometry. Repeating
            # union on its many disconnected parts can consume a control tick.
            geometry = live[0][1] if len(live) == 1 else unary_union([g for _, g in live])
            self._geometry_cache = (key, geometry)
        return self._geometry_cache[1]


@dataclass
class Params:
    radius: float = 0.22
    margin: float = 0.08
    speed: float = 0.35
    max_omega: float = 1.0
    acceleration: float = 0.5
    angular_acceleration: float = 1.5
    horizon: float = 3.0
    dt: float = 0.1
    goal_tolerance: float = 0.15
    switch_score_margin: float = 0.20
    min_turn_radius: float = 0.0
    prefer_reference: bool = False
    curvature_rate: float = 0.6
    search_period: float = 0.0
    braking_deceleration: float | None = None
    cruise_speed: float = 0.0
    swept_collision_check: bool = False
    cross_track_weight: float = 1.3
    offset_weight: float = .4
    semantic_weight: float = .25
    relaxed_rejoin: bool = False
    lookahead_distances: tuple = (1., 1.5)
    lateral_offsets: tuple = (0., -.15, .15, -.30, .30, -.45, .45, -.60, .60)

    def __post_init__(self):
        if self.braking_deceleration is None: self.braking_deceleration = self.acceleration
        if not np.isfinite(self.braking_deceleration) or self.braking_deceleration <= 0:
            raise ValueError('Invalid braking deceleration')
        if not np.isfinite(self.cruise_speed) or self.cruise_speed < 0:
            raise ValueError('Invalid cruise speed')
        positive = (self.radius, self.speed, self.max_omega, self.acceleration,
                    self.angular_acceleration, self.horizon, self.dt, self.goal_tolerance)
        if not all(np.isfinite(v) and v > 0
                   for v in positive) or not np.isfinite(self.margin) or self.margin < 0:
            raise ValueError('Invalid planner dimensions or timing')
        if not np.isfinite(self.switch_score_margin) or self.switch_score_margin < 0:
            raise ValueError('Invalid trajectory switching margin')
        if not np.isfinite(self.min_turn_radius) or self.min_turn_radius < 0:
            raise ValueError('Invalid minimum turn radius')
        if not np.isfinite(self.curvature_rate) or self.curvature_rate <= 0:
            raise ValueError('Invalid curvature slew rate')
        if not np.isfinite(self.search_period) or self.search_period < 0:
            raise ValueError('Invalid trajectory search period')
        if any(not np.isfinite(v) or v < 0 for v in (self.cross_track_weight,self.offset_weight,self.semantic_weight)):
            raise ValueError('Invalid reference attraction weight')
        if not self.lookahead_distances or any(not np.isfinite(v) or v<=0 for v in self.lookahead_distances):
            raise ValueError('Invalid lookahead distances')
        if not self.lateral_offsets or not np.isfinite(self.lateral_offsets).all():
            raise ValueError('Invalid lateral offsets')


@dataclass
class Plan:
    command: tuple
    trajectory: np.ndarray
    reason: str
    legal_count: int = 0
    clearance: float = 0.0
    offset: float = 0.0
    candidate: tuple | None = None
    selection_reason: str = 'NONE'
    challenger_gain: float = 0.0
    constraints: dict = field(default_factory=dict)


class Reference:

    def __init__(self, xy):
        xy = np.asarray(xy, dtype=float)
        if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2 or not np.isfinite(xy).all():
            raise ValueError('Reference needs at least two finite XY points')
        xy = xy[np.r_[True, np.linalg.norm(np.diff(xy, axis=0), axis=1) > 1e-6]]
        if len(xy) < 2:
            raise ValueError('Zero length reference')
        self.line = LineString(xy)
        self.length = self.line.length
        self.s = np.linspace(0, self.length, max(2, math.ceil(self.length / .04) + 1))
        self.xy = np.array([self.line.interpolate(s).coords[0] for s in self.s])
        delta = np.gradient(self.xy, axis=0)
        self.normal = np.column_stack((-delta[:, 1], delta[:, 0]))
        self.normal /= np.maximum(np.linalg.norm(self.normal, axis=1)[:, None], 1e-9)
        self.tree = cKDTree(self.xy)
        self.progress_index = 0

    def track(self, xy, window=2., max_cross_track=2.):
        lo = max(0, int(np.searchsorted(self.s, self.s[self.progress_index] - .5)))
        hi = min(len(self.xy),
                 int(np.searchsorted(self.s, self.s[self.progress_index] + window)) + 1)
        distances = np.linalg.norm(self.xy[lo:hi] - np.asarray(xy), axis=1)
        index = lo + int(np.argmin(distances))
        if float(np.min(distances)) > max_cross_track:
            raise ValueError('Outside route association corridor')
        self.progress_index = max(self.progress_index, index)
        return self.progress_index

    def station(self, xy):
        lo = max(0, self.progress_index - 15)
        hi = min(len(self.xy), self.progress_index + 100)
        return lo + np.argmin(np.linalg.norm(self.xy[lo:hi] - np.asarray(xy), axis=1))


def boundary_distances(road, points):
    """Exact geometry-to-boundary distances via an index of boundary segments."""
    parts = shapely.get_parts(road.boundary)
    coords, part_ids = shapely.get_coordinates(parts, return_index=True)
    same_part = part_ids[:-1] == part_ids[1:]
    if not np.any(same_part):
        return shapely.distance(road.boundary, points)
    # Never join the end of one boundary ring to the start of another.
    segments = np.stack((coords[:-1][same_part], coords[1:][same_part]), axis=1)
    tree = shapely.STRtree(shapely.linestrings(segments))
    indices, distances = tree.query_nearest(points, return_distance=True, all_matches=False)
    result = np.full(len(points), np.inf)
    result[indices[0]] = distances
    return result


def safe_trajectory(poses, road, obstacles, radius, max_step):
    """Sampling padding bounds continuous swept disk between trajectory samples."""
    xy = np.asarray(poses)[:, :2]
    pts = shapely.points(xy)
    # Any point along the arc is within max_step/2 of a sample by arc length.
    required = radius + max_step / 2 + 1e-6
    if road.is_empty or not np.all(shapely.covers(road, pts)):
        return False, 0.0
    clearance = float(np.min(shapely.distance(road.boundary, pts)))
    if obstacles is not None:
        clearance = min(clearance, float(np.min(obstacles.query(xy)[0])))
    return clearance >= required, clearance


def swept_shapes(poses):
    """Polyline through exact integration samples; stationary paths are points."""
    xy = np.asarray(poses)[:, :, :2]
    if xy.shape[1] == 1: return shapely.points(xy[:, 0])
    shapes = shapely.linestrings(xy)
    stationary = np.all(xy == xy[:, 0:1], axis=(1, 2))
    shapes[stationary] = shapely.points(xy[stationary, 0])
    return shapes


def obstacle_path_distances(shapes, obstacles):
    result = np.full(len(shapes), np.inf)
    if obstacles is None or not len(obstacles.data): return result
    index = shapely.STRtree(shapely.points(obstacles.data))
    ids, distances = index.query_nearest(shapes, return_distance=True, all_matches=False)
    result[ids[0]] = distances
    return result


def safe_swept_trajectory(poses, road, obstacles, radius, arc_error):
    shapes = swept_shapes(np.asarray(poses)[None])
    if road.is_empty or not shapely.covers(road, shapes)[0]: return False, 0.
    clearance = float(
        min(boundary_distances(road, shapes)[0],
            obstacle_path_distances(shapes, obstacles)[0]))
    return clearance >= radius + arc_error + 1e-6, clearance


class Planner:

    def __init__(self, params=None):
        self.p = params or Params()
        self.reset_selection()

    def reset_selection(self):
        self.selection_reference = None
        self.selected_candidate = None
        self.aligning = False
        self.last_search = -np.inf

    def stopped(self, pose, reason, clearance=0., constraints=None):
        self.reset_selection()
        return Plan((0., 0.),
                    pose[None],
                    reason,
                    clearance=clearance,
                    constraints=constraints or {})

    def safe_deceleration(self, pose, road, tree, previous, clearance):
        """Execute one braking step only if its complete stopping sweep is safe.

        This uses current geometry and obstacles, never a cached certificate.
        It is called only after the controller's freshness/validity checks.
        """
        p = self.p
        v, w = previous
        if v <= 1e-5:
            return self.stopped(pose, 'BLOCKED', clearance)
        curvature = w / v
        if p.min_turn_radius > 0 and abs(curvature) > 1. / p.min_turn_radius + 1e-9:
            return self.stopped(pose, 'BLOCKED', clearance)
        poses = [pose.copy()]
        commands = []
        while v > 1e-5 or abs(w) > 1e-5:
            v = max(0., v - p.braking_deceleration * p.dt)
            w = curvature * v if p.min_turn_radius > 0 else w - float(
                np.clip(w, -p.angular_acceleration * p.dt, p.angular_acceleration * p.dt))
            poses.append(integrate(poses[-1], v, w, p.dt))
            commands.append((v, w))
        trajectory = np.asarray(poses)
        if p.swept_collision_check:
            error = previous[0] * max(p.max_omega, abs(previous[1])) * p.dt**2 / 8
            safe, brake_clearance = safe_swept_trajectory(trajectory, road, tree,
                                                          p.radius + p.margin, error)
        else:
            safe, brake_clearance = safe_trajectory(trajectory, road, tree, p.radius + p.margin,
                                                    previous[0] * p.dt)
        if not safe:
            return self.stopped(pose, 'BLOCKED', clearance)
        # Keep the incumbent parameters for recovery, but revalidate them on
        # the next tick. No old road or old trajectory is reused.
        return Plan(commands[0],
                    trajectory,
                    'ACTIVE',
                    1,
                    brake_clearance,
                    candidate=None,
                    selection_reason='SAFE_DECELERATION')

    def plan(self,
             pose,
             reference,
             road,
             obstacle_xy,
             previous=(0., 0.),
             *,
             now=None,
             semantic_road=None,
             _force_search=False,
             _crawl_search=False):
        p = self.p
        now = time.monotonic() if now is None else float(now)
        if self.selection_reference is not reference:
            self.reset_selection()
            self.selection_reference = reference
        pose = np.asarray(pose, dtype=float)
        obstacles = np.asarray(obstacle_xy, dtype=float).reshape(-1, 2)
        if not np.isfinite(pose).all() or not np.isfinite(obstacles).all():
            return self.stopped(pose, 'INVALID_GEOMETRY')
        tree = cKDTree(obstacles) if len(obstacles) else None
        radius = p.radius + p.margin
        safe, clearance = safe_trajectory(pose[None], road, tree, radius, 0.)
        if not safe:
            road_clearance = float(road.boundary.distance(Point(
                pose[:2]))) if not road.is_empty else 0.
            obstacle_clearance = float(tree.query(pose[:2])[0]) if tree is not None else None
            return self.stopped(
                pose, 'NO_SAFE_START', clearance,
                dict(required_clearance_m=radius + 1e-6,
                     road_clearance_m=road_clearance,
                     road_start_rejected=not road.covers(Point(pose[:2]))
                     or road_clearance < radius + 1e-6,
                     obstacle_clearance_m=obstacle_clearance,
                     obstacle_start_rejected=obstacle_clearance is not None
                     and obstacle_clearance < radius + 1e-6))
        try:
            start_i = reference.track(pose[:2])
        except ValueError:
            return self.stopped(pose, 'OFF_ROUTE')
        if reference.length - reference.s[start_i] <= p.goal_tolerance and np.linalg.norm(
                pose[:2] - reference.xy[-1]) <= p.goal_tolerance:
            return self.stopped(pose, 'ARRIVED', clearance)
        if p.min_turn_radius > 0:
            target_i = min(
                len(reference.xy) - 1, int(np.searchsorted(reference.s,
                                                           reference.s[start_i] + 1.2)))
            delta = reference.xy[target_i] - pose[:2]
            heading_error = float(wrap(math.atan2(delta[1], delta[0]) - pose[2]))
            # Deliberate alignment is separate from driving. The circular
            # footprint was checked above and covers every in-place heading.
            if abs(heading_error) > 1.0: self.aligning = True
            if self.aligning and abs(heading_error) < .15: self.aligning = False
            if self.aligning:
                self.selected_candidate = None
                target_omega = float(
                    np.clip(1.2 * heading_error, -min(.3, p.max_omega), min(.3, p.max_omega)))
                omega = float(previous[1] +
                              np.clip(target_omega - previous[1], -.6 * p.dt, .6 * p.dt))
                return Plan((0., omega), pose[None], 'ACTIVE', 1, clearance, 0., None,
                            'ALIGN_HEADING')
        # Keep lower-speed rollouts when the current observed road cannot
        # contain the fast rollout. All candidates retain full braking checks.
        speeds = sorted(
            {
                p.speed, p.speed * .55,
                min(p.speed, .30),
                min(p.speed, .165),
                min(p.speed, .10),
                min(p.speed, .05),
                min(p.speed, .0275)
            },
            reverse=True)
        if p.cruise_speed > 0:
            speeds = sorted(
                {
                    min(p.speed, v)
                    for v in ((.10, .05, .0275) if _crawl_search else (p.speed, p.cruise_speed,
                                                                       p.cruise_speed * .9,
                                                                       p.cruise_speed * .8,
                                                                       p.cruise_speed * .7,
                                                                       p.cruise_speed * .4))
                },
                reverse=True)
        candidates = np.array([(o, l, v) for o in p.lateral_offsets
                               for l in (p.lookahead_distances if p.min_turn_radius > 0 else (.35, .70))
                               for v in speeds])
        # Tracking still recomputes and checks the ENTIRE incumbent rollout
        # against this tick's pose, road and obstacles. Only the expensive
        # comparison with alternatives runs at the slower search cadence.
        tracking = (not _force_search and self.selected_candidate is not None
                    and 0 <= now - self.last_search < p.search_period)
        if tracking:
            candidates = np.asarray([self.selected_candidate])
        else:
            self.last_search = now
        offset, lookahead, speed = candidates.T
        count = len(candidates)
        q = np.tile(pose, (count, 1))
        v = np.full(count, previous[0])
        w = np.full(count, previous[1])
        curvature = np.full(count, previous[1] / previous[0] if previous[0] > 1e-5 else 0.)
        index = np.full(count, start_i, dtype=int)
        poses = [q.copy()]
        commands = []
        offsets = np.arange(-3, 45)

        def step(q, v, w):
            yaw = q[:, 2]
            end = yaw + w * p.dt
            den = np.where(np.abs(w) > 1e-8, w, 1.)
            dx = np.where(
                np.abs(w) > 1e-8, v / den * (np.sin(end) - np.sin(yaw)), v * p.dt * np.cos(yaw))
            dy = np.where(
                np.abs(w) > 1e-8, v / den * (np.cos(yaw) - np.cos(end)), v * p.dt * np.sin(yaw))
            return np.column_stack((q[:, 0] + dx, q[:, 1] + dy, wrap(end)))

        for _ in range(math.ceil(p.horizon / p.dt)):
            ids = np.clip(index[:, None] + offsets, max(0, start_i - 2), len(reference.xy) - 1)
            distance = np.sum((reference.xy[ids] - q[:, None, :2])**2, axis=2)
            index = ids[np.arange(count), np.argmin(distance, axis=1)]
            target_i = np.minimum(
                len(reference.xy) - 1, np.searchsorted(reference.s, reference.s[index] + lookahead))
            target = reference.xy[target_i] + offset[:, None] * reference.normal[target_i]
            angle = wrap(np.arctan2(target[:, 1] - q[:, 1], target[:, 0] - q[:, 0]) - q[:, 2])
            # Along-route station can reach the endpoint while the robot is
            # still laterally displaced. Brake by actual goal distance too.
            remaining = np.maximum(reference.length - reference.s[index],
                                   np.linalg.norm(q[:, :2] - reference.xy[-1], axis=1))
            desired_v = speed * np.maximum(0., np.cos(angle))**2 * np.minimum(1., remaining / .4)
            desired_w = np.clip((2. * desired_v * np.sin(angle) /
                                 lookahead if p.min_turn_radius > 0 else 2. * angle), -p.max_omega,
                                p.max_omega)
            v += np.clip(desired_v - v, -p.braking_deceleration * p.dt, p.acceleration * p.dt)
            if p.min_turn_radius > 0:
                desired_curvature = np.clip(desired_w / np.maximum(desired_v, 1e-8),
                                            -1. / p.min_turn_radius, 1. / p.min_turn_radius)
                curvature += np.clip(desired_curvature - curvature, -p.curvature_rate * p.dt,
                                     p.curvature_rate * p.dt)
                curvature_limit = np.minimum(1. / p.min_turn_radius,
                                             p.max_omega / np.maximum(v, 1e-8))
                curvature = np.clip(curvature, -curvature_limit, curvature_limit)
                w = curvature * v
            else:
                w += np.clip(desired_w - w, -p.angular_acceleration * p.dt,
                             p.angular_acceleration * p.dt)
            q = step(q, v, w)
            poses.append(q.copy())
            commands.append(np.column_stack((v, w)))
        trajectories = np.stack(poses, axis=1)
        controls = np.stack(commands, axis=1)
        brake = []
        while np.any(v > 1e-5) or np.any(np.abs(w) > 1e-5):
            v = np.maximum(0., v - p.braking_deceleration * p.dt)
            if p.min_turn_radius > 0: w = curvature * v
            else: w -= np.clip(w, -p.angular_acceleration * p.dt, p.angular_acceleration * p.dt)
            q = step(q, v, w)
            brake.append(q.copy())
        checked = np.concatenate(
            (trajectories, np.stack(brake, axis=1)), axis=1) if brake else trajectories
        shapely.prepare(road)
        if p.swept_collision_check:
            shapes = swept_shapes(checked)
            inside = shapely.covers(road, shapes)
            road_clear = boundary_distances(road, shapes)
            obstacle_clear = obstacle_path_distances(shapes, tree)
            # Constant-twist motion between samples differs from its chord by
            # at most max(|v*w|)*dt^2/8 (linear-interpolation error bound).
            # Check complete chords, then cover their arc error. A speed*dt/2
            # disk around EVERY sample falsely rejected the nearfield start.
            required = radius + np.maximum(speed, previous[0]) * max(p.max_omega, abs(
                previous[1])) * p.dt**2 / 8 + 1e-6
        else:
            pts = shapely.points(checked[:, :, :2].reshape(-1, 2))
            inside = shapely.covers(road, pts).reshape(count, -1).all(axis=1)
            road_clear = boundary_distances(road, shapely.multipoints(checked[:, :, :2]))
            obstacle_clear = np.full(count, np.inf)
            if tree is not None:
                obstacle_clear = tree.query(checked[:, :, :2].reshape(-1,
                                                                      2))[0].reshape(count,
                                                                                     -1).min(axis=1)
            required = radius + np.maximum(speed, previous[0]) * p.dt / 2 + 1e-6
        clear = np.minimum(road_clear, obstacle_clear)
        road_valid = inside & (road_clear >= required)
        obstacle_valid = obstacle_clear >= required
        valid = road_valid & obstacle_valid
        legal = int(valid.sum())
        speed_checks = [
            dict(speed_mps=float(s),
                 candidates=int((speed == s).sum()),
                 road_legal=int((road_valid & (speed == s)).sum()),
                 obstacle_legal=int((obstacle_valid & (speed == s)).sum()),
                 jointly_legal=int((valid & (speed == s)).sum()))
            for s in sorted(set(speed), reverse=True)
        ]
        if not legal:
            if tracking:
                return self.plan(pose,
                                 reference,
                                 road,
                                 obstacle_xy,
                                 previous,
                                 now=now,
                                 semantic_road=semantic_road,
                                 _force_search=True)
            if p.cruise_speed > 0 and not _crawl_search:
                result = self.plan(pose,
                                   reference,
                                   road,
                                   obstacle_xy,
                                   previous,
                                   now=now,
                                   semantic_road=semantic_road,
                                   _force_search=True,
                                   _crawl_search=True)
                result.constraints['cruise_speed_checks'] = speed_checks
                return result
            result = self.safe_deceleration(pose, road, tree, previous, clearance)
            result.constraints = dict(candidates_checked=count,
                                      road_rejected=int((~road_valid).sum()),
                                      obstacle_rejected=int((~obstacle_valid).sum()),
                                      jointly_legal=legal,
                                      braking_safe=result.reason == 'ACTIVE',
                                      speed_checks=speed_checks)
            return result
        progress = reference.s[index] - reference.s[start_i]
        cross_track = reference.tree.query(trajectories[:, -1, :2])[0]
        score = 5. * progress - p.cross_track_weight * cross_track - p.offset_weight * np.abs(offset) + .12 * np.minimum(
            clear - radius, .5)
        score -= .03 * np.mean(np.abs(np.diff(controls[:, :, 1], axis=1)), axis=1)
        # Geometry remains the sole motion certificate. Semantics only adds a
        # bounded preference and can never rescue an unsafe path.
        semantic_bonus = np.zeros(count)
        if semantic_road is not None and not semantic_road.is_empty:
            sample = shapely.points(trajectories[:, :, :2].reshape(-1, 2))
            semantic_bonus = p.semantic_weight*np.mean(
                shapely.covers(semantic_road, sample).reshape(count, -1), axis=1)
        score += semantic_bonus
        score[~valid] = -np.inf
        preferred = valid & (np.abs(offset) < 1e-9) if p.prefer_reference else np.zeros(count,
                                                                                        dtype=bool)
        rank = score.copy()
        if preferred.any(): rank[~preferred] = -np.inf
        best = int(np.argmax(rank))
        selection_reason = 'TRACK_REVALIDATED' if tracking else 'FIRST_SELECTION'
        gain = 0.
        if self.selected_candidate is not None:
            matches = np.flatnonzero(
                np.all(np.isclose(candidates, self.selected_candidate, rtol=0., atol=1e-12),
                       axis=1))
            if len(matches) and valid[int(matches[0])] and (not preferred.any()
                                                            or preferred[int(matches[0])]):
                incumbent = int(matches[0])
                gain = float(score[best] - score[incumbent])
                if gain <= p.switch_score_margin:
                    best = incumbent
                    selection_reason = 'TRACK_REVALIDATED' if tracking else 'KEEP_SAFE_SELECTION'
                else:
                    selection_reason = 'SWITCH_BETTER'
            elif len(matches) and valid[int(matches[0])]:
                selection_reason = 'SWITCH_REFERENCE_PRIORITY'
            else:
                selection_reason = 'SWITCH_UNSAFE_OR_UNAVAILABLE'
        # Store only the selected rollout parameters, never old road evidence
        # or an unchecked old path. Every cycle regenerated and checked it above.
        self.selected_candidate = tuple(map(float, candidates[best]))
        return Plan(tuple(map(float, controls[best, 0])),
                    trajectories[best],
                    'ACTIVE',
                    legal,
                    float(clear[best]),
                    float(offset[best]),
                    self.selected_candidate,
                    selection_reason,
                    gain,
                    constraints=dict(
                        speed_checks=speed_checks,
                        semantic_policy='SOFT_PREFERENCE' if semantic_road is not None else 'LEGACY_HARD_REGION',
                        semantic_bonus=float(semantic_bonus[best]),
                        crawl_fallback=bool(
                            _crawl_search
                            or (p.cruise_speed > 0
                                and speed[best] < min(p.speed, p.cruise_speed * .4) - 1e-9)),
                        swept_collision_check=p.swept_collision_check))


def detect_low_obstacles(points,
                         road_labels,
                         *,
                         min_height=.04,
                         max_height=1.5,
                         ground_tolerance=.018,
                         max_slope_deg=15.,
                         min_cluster=4,
                         detect_obstacles=True):
    """Robust local ground z=ax+by+c; protrusions survive even if labelled road."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    labels = np.asarray(road_labels)
    finite = np.isfinite(points).all(axis=1)
    points, labels = points[finite], labels[finite]
    roi = (points[:, 0] > .15) & (points[:, 0] < 5.) & (np.abs(points[:, 1]) < 2.5)
    points, labels = points[roi], labels[roi]
    ground_candidates = points[(labels == 1) & (points[:, 2] > -.6) & (points[:, 2] < .15)]
    if len(ground_candidates) < 40:
        return np.empty((0, 3)), {'valid': False, 'reason': 'INSUFFICIENT_GROUND_SUPPORT'}
    rng = np.random.default_rng(17)
    fit = ground_candidates[::max(1, len(ground_candidates) // 1800)]
    a = np.column_stack((fit[:, :2], np.ones(len(fit))))
    best = None
    for _ in range(90):
        ids = rng.choice(len(fit), 3, replace=False)
        try:
            model = np.linalg.solve(a[ids], fit[ids, 2])
        except np.linalg.LinAlgError:
            continue
        if np.linalg.norm(model[:2]) > np.tan(np.deg2rad(max_slope_deg)):
            continue
        residual = np.abs(a @ model - fit[:, 2])
        inliers = residual < ground_tolerance
        n = int(inliers.sum())
        if best is None or n > best[0]:
            best = (n, inliers)
    if best is None or best[0] < max(35, len(fit) * .5):
        return np.empty((0, 3)), {'valid': False, 'reason': 'GROUND_FIT_UNCERTAIN'}
    support = fit[best[1]]
    if np.ptp(support[:, 0]) < .5 or np.ptp(support[:, 1]) < .25:
        return np.empty((0, 3)), {'valid': False, 'reason': 'GROUND_SUPPORT_TOO_NARROW'}
    model = np.linalg.lstsq(a[best[1]], fit[best[1], 2], rcond=None)[0]
    ground_error = support[:, 2] - np.column_stack((support[:, :2], np.ones(len(support)))) @ model
    noise = float(1.4826 * np.median(np.abs(ground_error - np.median(ground_error))))
    threshold = max(min_height, 3. * noise)
    if not detect_obstacles:
        return np.empty((0, 3)), {
            'valid': True,
            'reason': 'OK',
            'plane_abc': model.tolist(),
            'ground_points': int(len(support)),
            'noise_m': noise,
            'height_threshold_m': threshold,
            'obstacle_points': 0,
            'obstacle_detection_enabled': False
        }
    # Do not extrapolate plane beyond observed ground support bounds.
    near_support = cKDTree(support[:, :2]).query(points[:, :2])[0] < .40
    heights = points[:, 2] - np.column_stack((points[:, :2], np.ones(len(points)))) @ model
    candidates = points[(heights > threshold) & (heights < max_height) & near_support]
    if len(candidates):
        # Require spatial support without removing a compact low block.
        tree = cKDTree(candidates[:, :2])
        supported = tree.query_ball_point(candidates[:, :2], .08, return_length=True) >= min_cluster
        candidates = candidates[supported]
    return candidates, {
        'valid': True,
        'reason': 'OK',
        'plane_abc': model.tolist(),
        'ground_points': int(len(support)),
        'noise_m': noise,
        'height_threshold_m': threshold,
        'obstacle_points': int(len(candidates)),
        'obstacle_detection_enabled': True
    }
