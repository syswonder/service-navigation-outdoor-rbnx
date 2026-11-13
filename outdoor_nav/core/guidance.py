import math
from bisect import bisect_left
from collections.abc import Sequence


def distance(a, b):
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def projection(point, points):
    best = None
    offset = 0
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        norm = dx * dx + dy * dy
        t = (
            max(
                0,
                min(
                    1, ((point["x"] - a["x"]) * dx + (point["y"] - a["y"]) * dy) / norm
                ),
            )
            if norm
            else 0
        )
        foot = {"x": a["x"] + t * dx, "y": a["y"] + t * dy}
        candidate = {
            "point": foot,
            "distance_m": distance(point, foot),
            "segment": i,
            "offset_m": offset + t * math.sqrt(norm),
        }
        if best is None or candidate["distance_m"] < best["distance_m"]:
            best = candidate
        offset += math.sqrt(norm)
    if best is None:
        raise ValueError("A polyline requires at least two points")
    return best


def validate_point(value):
    if not isinstance(value, dict) or not all(
        isinstance(value.get(k), (int, float))
        and not isinstance(value[k], bool)
        and math.isfinite(value[k])
        for k in ("x", "y")
    ):
        raise ValueError("A map point requires finite x and y in meters")
    return {k: float(value[k]) for k in ("x", "y")}


def free_via_point(point, grid, radius_m=1.0):
    """Move an occupied temporary station within 1 m; never invent free space.

    The collision-aware planner still has to connect this candidate. Final
    destinations must not be passed through this function.
    """
    if grid is None:
        return point
    meta = grid.info
    resolution = meta.resolution
    col = math.floor((point["x"] - meta.origin.position.x) / resolution)
    row = math.floor((point["y"] - meta.origin.position.y) / resolution)

    def free(x, y):
        return (0 <= x < meta.width and 0 <= y < meta.height
                and 0 <= grid.data[y * meta.width + x] < 50)

    if free(col, row):
        return point
    cells = math.ceil(radius_m / resolution)
    best, best_distance = point, math.inf
    for y in range(max(0, row-cells), min(meta.height, row+cells+1)):
        for x in range(max(0, col-cells), min(meta.width, col+cells+1)):
            if not free(x, y):
                continue
            candidate = {"x": meta.origin.position.x + (x+.5)*resolution,
                         "y": meta.origin.position.y + (y+.5)*resolution}
            delta = distance(point, candidate)
            if delta <= radius_m and delta < best_distance:
                best, best_distance = candidate, delta
    return best


def observed_obstacle(point, grid, robot, now_s, timeout_s=3.0, scan=None):
    """Only known occupied cells in the live lidar's six-metre marking range."""
    if grid is None or grid.header.frame_id != "map":
        return False
    stamp = grid.header.stamp.sec + grid.header.stamp.nanosec / 1e9
    if not -0.1 <= now_s-stamp < timeout_s or distance(point, robot) > 6.0:
        return False
    if scan is not None:
        # Confirm a current return near this station, rather than skip on an
        # old mark elsewhere in the rolling map. Live scans use base_link.
        if scan.header.frame_id != 'base_link':
            return False
        dx, dy = point['x']-robot['x'], point['y']-robot['y']
        co, si = math.cos(robot['yaw']), math.sin(robot['yaw'])
        px, py = co*dx+si*dy, -si*dx+co*dy
        hit = False
        for i, value in enumerate(scan.ranges):
            if not (math.isfinite(value) and scan.range_min <= value <= scan.range_max):
                continue
            beam = scan.angle_min+i*scan.angle_increment
            if math.hypot(value*math.cos(beam)-px, value*math.sin(beam)-py) <= .8:
                hit = True
                break
        if not hit:
            return False
    meta = grid.info
    q = meta.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    dx, dy = point['x']-meta.origin.position.x, point['y']-meta.origin.position.y
    col = math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/meta.resolution)
    row = math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/meta.resolution)
    return (0 <= col < meta.width and 0 <= row < meta.height
            and grid.data[row*meta.width+col] >= 99)


class SampledPolyline(Sequence):
    """Two-metre route stations computed on demand from the retained corners."""

    def __init__(self, corners):
        self.corners = corners
        self.indices = [0]
        for a, b in zip(corners, corners[1:]):
            self.indices.append(self.indices[-1] + max(1, math.ceil(distance(a, b) / 2.0)))
        self.remaining = [0.0] * len(corners)
        for i in range(len(corners) - 2, -1, -1):
            self.remaining[i] = distance(corners[i], corners[i + 1]) + self.remaining[i + 1]

    def __len__(self):
        return self.indices[-1] + 1

    def __getitem__(self, index):
        if isinstance(index, slice):
            return [self[i] for i in range(*index.indices(len(self)))]
        if index < 0:
            index += len(self)
        if not 0 <= index < len(self):
            raise IndexError(index)
        end = bisect_left(self.indices, index)
        if self.indices[end] == index:
            return dict(self.corners[end])
        a, b = self.corners[end - 1:end + 1]
        ratio = (index - self.indices[end - 1]) / (self.indices[end] - self.indices[end - 1])
        return {key: a[key] + (b[key] - a[key]) * ratio for key in ("x", "y")}

    def remaining_length(self, index):
        end = bisect_left(self.indices, index)
        return distance(self[index], self.corners[end]) + self.remaining[end]


class RouteGuidance:
    """Ordered, short planning legs; road geometry is guidance, never map data.

    Simplify survey noise before subdividing long edges. Progress only visits the
    next leg, never the globally nearest road (which can be a later parallel arm
    of a U). A rolling window has several legs, so RPP drives through the via
    points rather than treating every road vertex as a stopping goal.
    """

    def __init__(self, reference, start, goal):
        points = []
        for point in [start, *reference, goal]:
            point = validate_point(point)
            if not points or distance(points[-1], point) > 1e-6:
                points.append(point)
        # Iterative Ramer-Douglas-Peucker, retaining bends to within 5 cm.
        keep = {0, len(points) - 1}
        pending = [(0, len(points) - 1)]
        while pending:
            a, b = pending.pop()
            if b - a < 2:
                continue
            errors = [
                projection(points[i], [points[a], points[b]])["distance_m"]
                for i in range(a + 1, b)
            ]
            offset = max(range(len(errors)), key=errors.__getitem__)
            if errors[offset] > 0.05:
                middle = a + 1 + offset
                keep.add(middle)
                pending.extend(((a, middle), (middle, b)))
        corners = [points[i] for i in sorted(keep)]
        self.points = SampledPolyline(corners)
        self.bends = set(self.points.indices[1:])
        self.index = min(1, len(self.points) - 1)
        self.skipped = set()

    @property
    def final_leg(self):
        return self.index == len(self.points) - 1

    def remaining_length(self, position):
        return distance(position, self.points[self.index]) + self.points.remaining_length(self.index)

    def window(self, position):
        """Return an ordered ~6 m horizon and its identity for preemption.

        Advance before the controller approaches its temporary endpoint. Crossing
        a station's end-plane also permits an obstacle detour up to 1.5 m away.
        Neither test can skip ahead to a spatially adjacent, later road segment.
        """
        while not self.final_leg:
            if self.index in self.skipped:
                self.index += 1
                continue
            a, b = self.points[self.index - 1 : self.index + 1]
            dx, dy = b["x"] - a["x"], b["y"] - a["y"]
            px, py = position["x"] - b["x"], position["y"] - b["y"]
            crossed = px * dx + py * dy >= 0
            lateral = abs(px * dy - py * dx) / math.hypot(dx, dy)
            if distance(position, b) > 0.6 and not (crossed and lateral <= 1.5):
                break
            self.index += 1
        end, ahead = self.index, distance(position, self.points[self.index])
        # Bound per-replan action calls even for extremely tightly spaced bends.
        while end + 1 < len(self.points) and end - self.index < 11:
            segment = distance(self.points[end], self.points[end + 1])
            if ahead + segment > 6.0 + 1e-6:
                break
            ahead += segment
            end += 1
        return tuple(range(self.index, end + 1))
