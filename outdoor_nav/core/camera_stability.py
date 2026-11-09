"""World-frame temporal evidence; each acquisition updates history at most once."""
import math


class StaticObstacles:
    def __init__(self, cell=0.15, hits=4, dwell=0.6, gap=0.35, drift=0.12):
        self.cell, self.hits, self.dwell, self.gap, self.drift = cell, hits, dwell, gap, drift
        self.tracks = []
        self.stamp = None

    def update(self, points, stamp):
        if not math.isfinite(stamp) or (self.stamp is not None and stamp <= self.stamp):
            if self.stamp is not None and stamp < self.stamp:
                self.tracks = []
                self.stamp = None
            return set()
        self.stamp = stamp
        cells = {}
        for i, (x, y) in enumerate(points):
            if math.isfinite(x) and math.isfinite(y):
                cells.setdefault((math.floor(x / self.cell), math.floor(y / self.cell)), []).append(i)
        # Connected components prevent a moving object from repeatedly promoting individual voxels.
        groups = []
        unseen = set(cells)
        while unseen:
            queue = [unseen.pop()]
            indices = []
            while queue:
                x, y = queue.pop()
                indices.extend(cells[x, y])
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)):
                    neighbor = (x + dx, y + dy)
                    if neighbor in unseen:
                        unseen.remove(neighbor)
                        queue.append(neighbor)
            groups.append(indices)
        previous = [t for t in self.tracks if 0 < stamp - t["last"] <= self.gap]
        candidates = []
        centers = [(sum(points[i][0] for i in g) / len(g), sum(points[i][1] for i in g) / len(g)) for g in groups]
        for a, center in enumerate(centers):
            for b, track in enumerate(previous):
                distance = math.dist(center, track["center"])
                if distance <= max(self.cell * 3, self.drift * 2):
                    candidates.append((distance, a, b))
        matched, used = {}, set()
        for _, a, b in sorted(candidates):
            if a not in matched and b not in used:
                matched[a] = previous[b]
                used.add(b)
        accepted, tracks = set(), []
        for a, (group, center) in enumerate(zip(groups, centers)):
            track = matched.get(a)
            if track is None or math.dist(center, track["anchor"]) > self.drift:
                track = {"anchor": center, "first": stamp, "hits": 0}
            track = {**track, "center": center, "last": stamp, "hits": track["hits"] + 1}
            tracks.append(track)
            if track["hits"] >= self.hits and stamp - track["first"] >= self.dwell:
                accepted.update(group)
        self.tracks = tracks
        return accepted
