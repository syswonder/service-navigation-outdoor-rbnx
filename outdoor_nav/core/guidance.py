import math


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
        self.points, self.bends = [corners[0]], set()
        for a, b in zip(corners, corners[1:]):
            steps = max(1, math.ceil(distance(a, b) / 2.0))
            self.points.extend(
                {key: a[key] + (b[key] - a[key]) * i / steps for key in ("x", "y")}
                for i in range(1, steps + 1)
            )
            self.points[-1] = dict(b)
            self.bends.add(len(self.points) - 1)
        self.index = min(1, len(self.points) - 1)

    @property
    def final_leg(self):
        return self.index == len(self.points) - 1

    def window(self, position):
        """Return an ordered ~6 m horizon and its identity for preemption.

        Advance before the controller approaches its temporary endpoint. Crossing
        a station's end-plane also permits an obstacle detour up to 1.5 m away.
        Neither test can skip ahead to a spatially adjacent, later road segment.
        """
        while not self.final_leg:
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
