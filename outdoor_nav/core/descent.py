"""Choose short observation goals and descent stages for Nav2."""

import math

import cv2
import numpy as np


class Survey:
    def __init__(self):
        self.visited = []
        self.failures = {}
        self.last_progress = None

    def moved(self, xy):
        if self.last_progress is None or math.dist(xy, self.last_progress) > .32:
            self.last_progress = xy
            self.failures.clear()

    def failed(self, point, resolution):
        key = self.key(point, resolution)
        self.failures[key] = self.failures.get(key, 0) + 1

    @staticmethod
    def key(point, resolution):
        return tuple(round(value / resolution) for value in point)

    @staticmethod
    def platform(heights, platform_z):
        return np.abs(heights - platform_z) <= .06

    def staged_candidates(self, support, heights, xy, goal, platform_z, current_z):
        if not support.clear([xy]):
            return []
        resolution = support.resolution
        x, y = np.floor((np.asarray(xy)-support.origin)/resolution).astype(int)
        _, labels = cv2.connectedComponents(support.safe.astype(np.uint8), connectivity=4)
        reachable = labels == labels[y, x]
        rows, cols = np.nonzero(reachable)
        coordinates = support.origin + (np.column_stack((cols, rows)) + .5) * resolution
        travel = np.linalg.norm(coordinates-xy, axis=1)
        remaining = np.linalg.norm(coordinates-goal, axis=1)
        clearance = cv2.distanceTransform(support.safe.astype(np.uint8), cv2.DIST_L2, 5)
        margin = clearance[rows, cols] * resolution
        z = heights[rows, cols]
        roomy = margin >= 1.4 * resolution

        # Once below the platform, level continuation at the foot of the
        # slope is still a descent stage. Do not require another drop there.
        lower = z <= min(platform_z - .10, current_z + .04)
        descent = lower & roomy & (travel >= .55) & (travel <= 1.9)
        descent &= math.dist(xy, goal) - remaining >= .28
        descent_score = remaining - .35 * np.minimum(margin, 1.2)

        upper = self.platform(heights, platform_z) & support.safe
        _, upper_labels = cv2.connectedComponents(upper.astype(np.uint8), connectivity=4)
        platform = upper[rows, cols] & (upper_labels[rows, cols] == upper_labels[y, x]) & upper[y, x]
        platform &= roomy & (travel >= .85) & (travel <= 3.25)
        novelty = np.full(len(rows), np.inf)
        for point in self.visited:
            novelty = np.minimum(novelty, np.linalg.norm(coordinates-point, axis=1))
        platform &= novelty >= .90
        platform_score = remaining + .12 * travel - .75 * np.minimum(novelty, 2.8)
        if lower.any():
            lead = coordinates[np.argmin(np.where(lower, remaining, np.inf))]
            platform_score += 1.1 * np.linalg.norm(coordinates-lead, axis=1)

        candidates = []
        if support.clear([goal]) and math.dist(xy, goal) <= 2.6:
            gx, gy = np.floor((np.asarray(goal)-support.origin)/resolution).astype(int)
            if reachable[gy, gx] and self.failures.get(self.key(goal, resolution), 0) < 3:
                candidates.append(("goal", tuple(goal)))
        for phase, eligible, score in (
            ("descent", descent, descent_score), ("platform", platform, platform_score)
        ):
            options = coordinates[eligible]
            for offset in np.argsort(score[eligible]):
                point = tuple(options[offset])
                if self.failures.get(self.key(point, resolution), 0) >= 3:
                    continue
                if all(math.dist(point, other) > .40 for _, other in candidates):
                    candidates.append((phase, point))
                if len(candidates) >= 8:
                    return candidates
        return candidates
