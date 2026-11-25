"""Bounded online ground observations and full-footprint support checks."""

import math
from dataclasses import dataclass

import cv2
import numpy as np

DEPTH_TOPIC = "/outdoor/ranger/registered_depth"
GRID_TOPIC = "/outdoor/ranger/ground_support"
HEIGHT_TOPIC = "/outdoor/ranger/ground_height"
ACK_TOPIC = "/outdoor/ranger/ground_support_ack"
DESCENT_ACTIVE_TOPIC = "/outdoor/ranger/descent_active"
CELL = .16
SIZE = 101
FRAME = "odom"
SEGMENT_SPEED = .22


def disk(radius, resolution):
    cells = math.ceil(radius / resolution)
    rows, cols = np.ogrid[-cells:cells+1, -cells:cells+1]
    return np.uint8((rows**2 + cols**2) * resolution**2 <= radius**2)


class GroundGrid:
    def __init__(self, step, grade, radius):
        self.step, self.grade, self.radius = step, grade, radius
        self.origin = None
        self.height = np.full((SIZE, SIZE), np.nan)
        self.observed = np.full((SIZE, SIZE), -np.inf)
        self.depth_observed = np.full_like(self.observed, -np.inf)
        self.rough = np.zeros((SIZE, SIZE), dtype=bool)
        self.hazard = np.zeros_like(self.rough)

    def reset(self, xy, ground, stamp):
        self.origin = np.asarray(xy) - (SIZE // 2 + .5) * CELL
        self.height.fill(np.nan)
        self.observed.fill(-np.inf)
        self.depth_observed.fill(-np.inf)
        self.rough.fill(False)
        # Seed only cells overlapping the vehicle's initial occupied footprint.
        footprint = disk(self.radius + CELL / math.sqrt(2), CELL).astype(bool)
        reach = footprint.shape[0] // 2
        sl = slice(SIZE//2-reach, SIZE//2+reach+1)
        self.height[sl, sl][footprint] = ground
        self.observed[sl, sl][footprint] = stamp

    def update(self, xy, ground, points, stamp, max_cell_age=45., depth_count=None, lidar_stamp=None):
        if self.origin is None:
            self.reset(xy, ground, stamp)
        # Recenter by whole cells, retaining only still-in-window observations.
        center = self.origin + (SIZE // 2 + .5) * CELL
        shift = np.rint((np.asarray(xy) - center) / CELL).astype(int)
        if np.max(np.abs(shift)) >= SIZE:
            self.height.fill(np.nan)
            self.observed.fill(-np.inf)
            self.depth_observed.fill(-np.inf)
            self.rough.fill(False)
            self.origin += shift * CELL
        elif np.max(np.abs(shift)) >= 12:
            dx, dy = shift
            for array, fill in ((self.height, np.nan), (self.observed, -np.inf),
                                (self.depth_observed, -np.inf), (self.rough, False)):
                array[:] = np.roll(array, (-dy, -dx), axis=(0, 1))
                if dy > 0:
                    array[-dy:] = fill
                elif dy < 0:
                    array[:-dy] = fill
                if dx > 0:
                    array[:, -dx:] = fill
                elif dx < 0:
                    array[:, :-dx] = fill
            self.origin += shift * CELL

        is_depth = np.arange(len(points)) < (len(points) if depth_count is None else depth_count)
        finite = np.isfinite(points).all(axis=1)
        points, is_depth = points[finite], is_depth[finite]
        planar = np.linalg.norm(points[:, :2] - xy, axis=1)
        visible = (planar > self.radius) & (planar < 7.) & (abs(points[:, 2] - ground) < 3.)
        points, is_depth = points[visible], is_depth[visible]
        ij = np.floor((points[:, :2] - self.origin) / CELL).astype(int)
        inside = ((ij >= 0) & (ij < SIZE)).all(axis=1)
        ij, z = ij[inside], points[inside, 2]
        flat = ij[:, 1] * SIZE + ij[:, 0]
        counts = np.bincount(flat, minlength=SIZE*SIZE)
        depth_cells = np.bincount(flat[is_depth[inside]], minlength=SIZE*SIZE) >= 2
        self.depth_observed.ravel()[depth_cells] = stamp
        groups = np.flatnonzero(counts >= 2)
        starts = np.cumsum(counts)[groups] - counts[groups]
        span = counts[groups] - 1
        trim = (span * .12).astype(int)
        ordered = z[np.lexsort((z, flat))]
        vertical = ordered[starts+span-trim] - ordered[starts+trim] > .11
        self.rough.ravel()[groups] = vertical
        self.observed.ravel()[groups] = (stamp if lidar_stamp is None else
                                         np.where(depth_cells[groups], stamp, min(stamp, lidar_stamp)))
        self.height.ravel()[groups] = .5 * (ordered[starts+span//2] + ordered[starts+(span+1)//2])

        rows, cols = np.indices(self.height.shape)
        near = np.hypot(self.origin[0]+(cols+.5)*CELL-xy[0],
                        self.origin[1]+(rows+.5)*CELL-xy[1])
        expired = (stamp - self.observed > max_cell_age) | (near > 7.5)
        # The occluded cells currently below the chassis already supported its
        # approach. Keep their actual heights while occupied.
        occupied = (near <= self.radius + CELL / math.sqrt(2)) & np.isfinite(self.height) & ~self.rough
        self.observed[occupied] = stamp
        expired[occupied] = False
        self.height[expired] = np.nan
        self.depth_observed[expired] = -np.inf
        self.rough[expired] = False
        hazard = self.rough.copy()
        for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)):
            a, b = self.bands(dx, dy, 1)
            step = abs(self.height[a] - self.height[b]) > self.step
            step &= ~self.rough[a] & ~self.rough[b]
            hazard[a] |= step
            hazard[b] |= step
            bands = self.bands(dx, dy, 3)
            heights = np.array([self.height[b] for b in bands])
            steep = (abs(np.diff(heights, axis=0)) <= self.step).all(axis=0)
            steep &= abs(heights[-1] - heights[0]) > self.grade * CELL * math.hypot(dx, dy) * 3 + .02
            hazard[bands[1]] |= steep
            hazard[bands[2]] |= steep
        self.hazard = hazard
        return np.isfinite(self.height) & ~hazard

    @staticmethod
    def bands(dx, dy, span):
        return [tuple(slice(max(0, -d*span) + d*k, SIZE-max(0, d*span) + d*k)
                      for d in (dy, dx)) for k in range(span+1)]


class SupportMask:
    def __init__(self, cells, origin, resolution, stamp, radius, hazards=None, depth_cells=None):
        self.cells, self.origin, self.resolution, self.stamp = cells, np.asarray(origin), resolution, stamp
        kernel = disk(radius + resolution / math.sqrt(2), resolution)
        self.safe = cv2.erode(cells.astype(np.uint8), kernel, borderType=cv2.BORDER_CONSTANT,
                              borderValue=0).astype(bool)
        self.hazard = (cv2.dilate(hazards.astype(np.uint8), kernel).astype(bool)
                       if hazards is not None else None)
        self.depth_cells = depth_cells

    def clear(self, points, allowed=None):
        points = np.asarray(points)
        if not np.isfinite(points).all():
            return False
        ij = np.floor((points - self.origin) / self.resolution).astype(int)
        x, y = ij.T
        height, width = self.safe.shape
        return bool(((x >= 0) & (x < width) & (y >= 0) & (y < height)).all()
                    and self.safe[y, x].all()
                    and (allowed is None or allowed[y, x].all()))

    def corridor(self, start, points, allowed=None):
        for end in points:
            count = max(2, math.ceil(math.dist(start, end) / (self.resolution * .25)) + 1)
            if not self.clear(np.linspace(start, end, count), allowed):
                return False
            start = end
        return True

    def profile(self, start, points, heights, limit=8., landing_distance=.70):
        """Sample the forward route up to the first unsupported footprint."""
        if heights is None or not self.clear([start]):
            return GroundProfile(0.)
        xy = np.asarray(start, dtype=float)
        distance, samples, levels = [0.], [xy], [self.level(xy, heights)]
        depth_seen = [self.depth_at(xy)]
        reason = "observed_path_ends"
        for end in points:
            end = np.asarray(end, dtype=float)
            length = math.dist(xy, end)
            if length <= 1e-6:
                continue
            count = math.ceil(length / (self.resolution * .25))
            for offset in np.linspace(0., 1., count + 1)[1:]:
                sample = xy + offset * (end - xy)
                traveled = distance[-1] + math.dist(samples[-1], sample)
                if traveled > limit or not self.clear([sample]):
                    reason = self.blocked_reason(sample) if traveled <= limit else reason
                    return ground_profile(distance, samples, levels, depth_seen, reason, landing_distance)
                distance.append(traveled)
                samples.append(sample)
                levels.append(self.level(sample, heights))
                depth_seen.append(self.depth_at(sample))
            xy = end
        return ground_profile(distance, samples, levels, depth_seen, reason, landing_distance)

    def level(self, xy, heights):
        x, y = np.floor((xy - self.origin) / self.resolution).astype(int)
        return float(heights[y, x])

    def depth_at(self, xy):
        if self.depth_cells is None:
            return True
        x, y = np.floor((xy - self.origin) / self.resolution).astype(int)
        return bool(self.depth_cells[y, x])

    def blocked_reason(self, xy):
        x, y = np.floor((xy - self.origin) / self.resolution).astype(int)
        if (self.hazard is not None and 0 <= y < self.safe.shape[0] and 0 <= x < self.safe.shape[1]
                and self.hazard[y, x]):
            return "ground_hazard"
        return "ground_unobserved"

    def permits(self, xy, yaw, speed, turn, measured_speed, config):
        if not all(map(math.isfinite, (*xy, yaw, speed, turn, measured_speed))):
            return False
        v = max(speed, abs(measured_speed))
        if speed == 0. and v < .01:
            return self.clear([xy])
        latency = config["ranger_depth_timeout_s"] + .05
        decel = config["ranger_brake_mps2"]
        distance = v * latency + v*v/(2*decel) + .04
        horizon = latency + v/decel
        steps = max(2, math.ceil(distance / (self.resolution * .25)) + 1)
        s = np.linspace(0, distance, steps)
        angle = yaw + np.linspace(0, turn*horizon, steps) * .5
        arc = np.column_stack((np.cos(angle), np.sin(angle))) * s[:, None] + xy
        straight = np.column_stack((np.cos(yaw)*s, np.sin(yaw)*s)) + xy
        return self.clear(arc) and self.clear(straight)


@dataclass(frozen=True)
class GroundProfile:
    supported_m: float
    crest_m: float | None = None
    crest: tuple | None = None
    landing: tuple | None = None
    reason: str = "ground_unobserved"


def ground_profile(distance, points, levels, depth_seen, reason, landing_distance):
    """Recognize a sustained drop followed by a contiguous flat landing."""
    distance, levels = np.asarray(distance), np.asarray(levels)
    crest = None
    last_drop = None
    depth_confirmed = False
    for index, far in enumerate(distance):
        near = np.searchsorted(distance, far - .55)
        drop = levels[near] - levels[index]
        if far >= .45 and drop >= .035:
            changes = -np.diff(levels[near:index + 1])
            gradual = (np.count_nonzero(changes >= .006) >= 2
                       and changes.max() <= max(.032, .75 * drop)
                       and not (changes < -.02).any())
            if gradual:
                if crest is None:
                    crest = near
                last_drop = index
                depth_confirmed |= depth_seen[near] and depth_seen[(near + index) // 2] and depth_seen[index]
            elif changes.max() >= .04:
                return GroundProfile(float(distance[near]), reason="ground_drop_discontinuous")
        if last_drop is not None and far - distance[last_drop] >= landing_distance:
            start = np.searchsorted(distance, far - landing_distance)
            if depth_confirmed and depth_seen[index] and np.ptp(levels[start:index + 1]) <= .035:
                return GroundProfile(float(distance[-1]), float(distance[crest]), tuple(points[crest]),
                                     tuple(points[index]), reason)
    if crest is None:
        return GroundProfile(float(distance[-1]), reason=reason)
    return GroundProfile(float(distance[-1]), float(distance[crest]), tuple(points[crest]), reason=reason)


def braking_speed(available, measured, requested, latency, deceleration):
    """Limit speed to a stopping corridor. Zero requests immediate braking."""
    distance = available - .04
    if distance <= 0. or measured * latency + measured**2 / (2 * deceleration) >= distance:
        return 0.
    cap = math.sqrt((deceleration * latency)**2 + 2 * deceleration * distance) - deceleration * latency
    return min(requested, cap)


def braking_paths(xy, yaw, speed, turn, measured, measured_turn, horizon):
    """Check the commanded turn, the measured turn while braking, and straight."""
    step = CELL * .25
    distance = np.linspace(0., horizon, math.ceil(horizon / step) + 1)
    measured_curvature = measured_turn / measured if measured > .01 else 0.
    for curvature in (turn / max(speed, .01), measured_curvature, 0.):
        if abs(curvature) < 1e-4:
            offsets = np.column_stack((np.cos(yaw) * distance, np.sin(yaw) * distance))
        else:
            angles = yaw + curvature * distance
            offsets = np.column_stack((np.sin(angles) - np.sin(yaw), np.cos(yaw) - np.cos(angles))) / curvature
        yield offsets[1:] + xy
