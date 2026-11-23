"""Bounded online ground observations and full-footprint support checks."""

import math

import cv2
import numpy as np

DEPTH_TOPIC = "/outdoor/ranger/registered_depth"
GRID_TOPIC = "/outdoor/ranger/ground_support"
ACK_TOPIC = "/outdoor/ranger/ground_support_ack"
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
        self.rough = np.zeros((SIZE, SIZE), dtype=bool)

    def reset(self, xy, ground, stamp):
        self.origin = np.asarray(xy) - (SIZE // 2 + .5) * CELL
        self.height.fill(np.nan)
        self.observed.fill(-np.inf)
        self.rough.fill(False)
        # Seed only cells overlapping the vehicle's initial occupied footprint.
        footprint = disk(self.radius + CELL / math.sqrt(2), CELL).astype(bool)
        reach = footprint.shape[0] // 2
        sl = slice(SIZE//2-reach, SIZE//2+reach+1)
        self.height[sl, sl][footprint] = ground
        self.observed[sl, sl][footprint] = stamp

    def update(self, xy, ground, points, stamp):
        if self.origin is None:
            self.reset(xy, ground, stamp)
        # Recenter by whole cells, retaining only still-in-window observations.
        center = self.origin + (SIZE // 2 + .5) * CELL
        shift = np.rint((np.asarray(xy) - center) / CELL).astype(int)
        if np.max(np.abs(shift)) >= SIZE:
            self.height.fill(np.nan)
            self.observed.fill(-np.inf)
            self.rough.fill(False)
            self.origin += shift * CELL
        elif np.max(np.abs(shift)) >= 12:
            dx, dy = shift
            for array, fill in ((self.height, np.nan), (self.observed, -np.inf), (self.rough, False)):
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

        points = points[np.isfinite(points).all(axis=1)]
        planar = np.linalg.norm(points[:, :2] - xy, axis=1)
        points = points[(planar > self.radius) & (planar < 7.) & (abs(points[:, 2] - ground) < 3.)]
        ij = np.floor((points[:, :2] - self.origin) / CELL).astype(int)
        inside = ((ij >= 0) & (ij < SIZE)).all(axis=1)
        ij, z = ij[inside], points[inside, 2]
        flat = ij[:, 1] * SIZE + ij[:, 0]
        counts = np.bincount(flat, minlength=SIZE*SIZE)
        groups = np.flatnonzero(counts >= 2)
        starts = np.cumsum(counts)[groups] - counts[groups]
        span = counts[groups] - 1
        trim = (span * .12).astype(int)
        ordered = z[np.lexsort((z, flat))]
        vertical = ordered[starts+span-trim] - ordered[starts+trim] > .11
        self.rough.ravel()[groups] = vertical
        self.observed.ravel()[groups] = stamp
        self.height.ravel()[groups] = .5 * (ordered[starts+span//2] + ordered[starts+(span+1)//2])

        rows, cols = np.indices(self.height.shape)
        near = np.hypot(self.origin[0]+(cols+.5)*CELL-xy[0],
                        self.origin[1]+(rows+.5)*CELL-xy[1])
        expired = (stamp - self.observed > 45.) | (near > 7.5)
        # The occluded cells currently below the chassis already supported its
        # approach. Keep their actual heights while occupied.
        occupied = (near <= self.radius + CELL / math.sqrt(2)) & np.isfinite(self.height) & ~self.rough
        self.observed[occupied] = stamp
        expired[occupied] = False
        self.height[expired] = np.nan
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
        return np.isfinite(self.height) & ~hazard

    @staticmethod
    def bands(dx, dy, span):
        return [tuple(slice(max(0, -d*span) + d*k, SIZE-max(0, d*span) + d*k)
                      for d in (dy, dx)) for k in range(span+1)]


class SupportMask:
    def __init__(self, cells, origin, resolution, stamp, radius):
        self.cells, self.origin, self.resolution, self.stamp = cells, np.asarray(origin), resolution, stamp
        kernel = disk(radius + resolution / math.sqrt(2), resolution)
        self.safe = cv2.erode(cells.astype(np.uint8), kernel, borderType=cv2.BORDER_CONSTANT,
                              borderValue=0).astype(bool)

    def clear(self, points):
        points = np.asarray(points)
        if not np.isfinite(points).all():
            return False
        ij = np.floor((points - self.origin) / self.resolution).astype(int)
        x, y = ij.T
        height, width = self.safe.shape
        return bool(((x >= 0) & (x < width) & (y >= 0) & (y < height)).all()
                    and self.safe[y, x].all())

    def corridor(self, start, points):
        for end in points:
            count = max(2, math.ceil(math.dist(start, end) / (self.resolution * .25)) + 1)
            if not self.clear(np.linspace(start, end, count)):
                return False
            start = end
        return True

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
