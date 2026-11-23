"""Select short observation goals; path planning is owned by Nav2."""

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

    def candidates(self, support, xy, goal):
        if not support.clear([xy]):
            return []
        resolution = support.resolution
        _, labels = cv2.connectedComponents(support.safe.astype(np.uint8), connectivity=4)
        x, y = np.floor((np.asarray(xy)-support.origin)/resolution).astype(int)
        reachable = labels == labels[y, x]
        rows, cols = np.nonzero(reachable)
        coordinates = support.origin + (np.column_stack((cols, rows)) + .5) * resolution
        travel = np.linalg.norm(coordinates-xy, axis=1)
        remaining = np.linalg.norm(coordinates-goal, axis=1)
        eligible = (travel >= .55) & (travel <= 1.9)
        for point in self.visited:
            eligible &= np.linalg.norm(coordinates-point, axis=1) >= .65
        clearance = cv2.distanceTransform(support.safe.astype(np.uint8), cv2.DIST_L2, 5)
        margin = clearance[rows, cols] * resolution
        eligible &= margin >= 1.4 * resolution
        score = remaining + .12 * travel - .85 * np.minimum(1., margin)
        candidates = [tuple(goal)] if support.clear([goal]) and math.dist(xy, goal) <= 2.6 else []
        options = coordinates[eligible]
        for offset in np.argsort(score[eligible]):
            point = options[offset]
            if all(math.dist(point, other) > .40 for other in candidates):
                candidates.append(tuple(point))
            if len(candidates) >= 8:
                break
        return [point for point in candidates if self.failures.get(self.key(point, resolution), 0) < 3]
