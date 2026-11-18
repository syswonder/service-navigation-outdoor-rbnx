"""Ordered sparse hints, not a path constraint or occupancy prior."""
import math
from .guidance import validate_point, distance, projection


def hint_indices(pool, spacing_m=6., corner_error_m=.3):
    """Preserve RDP corners, then take original samples at >= spacing per leg.

    Corners and the final point are mandatory exceptions to minimum spacing.
    Indices preserve survey order even on loops or adjacent parallel roads.
    """
    if spacing_m < 6 or corner_error_m <= 0 or not pool:
        raise ValueError('Hints require spacing >= 6 m and positive corner tolerance')
    points = [validate_point(p) for p in pool]
    corners = {0, len(points)-1}
    pending = [(0, len(points)-1)]
    while pending:
        a,b = pending.pop()
        if b-a < 2:
            continue
        errors = [projection(points[i], [points[a], points[b]])['distance_m'] for i in range(a+1,b)]
        offset = max(range(len(errors)), key=errors.__getitem__)
        if errors[offset] > corner_error_m:
            mid = a+1+offset
            corners.add(mid)
            pending.extend(((a,mid),(mid,b)))
    result = [0]
    for end in sorted(corners)[1:]:
        for index in range(result[-1]+1, end):
            if distance(points[result[-1]], points[index]) >= spacing_m:
                result.append(index)
        if result[-1] != end:
            result.append(end)
    return result, corners


def map_target_in_odom(target, map_pose, odom_pose):
    """T_odom_base * inverse(T_map_base) * target, for a time-paired pose."""
    angle = odom_pose['yaw']-map_pose['yaw']
    c,s = math.cos(angle), math.sin(angle)
    dx,dy = target['x']-map_pose['x'], target['y']-map_pose['y']
    return dict(x=odom_pose['x']+c*dx-s*dy, y=odom_pose['y']+s*dx+c*dy)


def body_delta(target, pose):
    dx,dy = target['x']-pose['x'], target['y']-pose['y']
    c,s = math.cos(pose['yaw']), math.sin(pose['yaw'])
    return c*dx+s*dy, -s*dx+c*dy


class HintQueue:
    def __init__(self, pool, spacing_m=6., corner_error_m=.3, arrival_side_m=3.):
        self.pool = [validate_point(p) for p in pool]
        self.indices, corners = hint_indices(self.pool, spacing_m, corner_error_m)
        self.corners = sorted(corners)
        self.index, self.half_side = 0, arrival_side_m/2
        if not math.isfinite(arrival_side_m) or arrival_side_m <= 0:
            raise ValueError('arrival_side_m must be positive')

    @property
    def target(self):
        return self.pool[self.indices[self.index]]

    @property
    def final(self):
        return self.index == len(self.indices)-1

    def advance(self, averaged_target, body_pose):
        x,y = body_delta(averaged_target, body_pose)
        arrived = abs(x) <= self.half_side and abs(y) <= self.half_side
        if arrived and not self.final:
            self.index += 1
            return True
        return False


def average_hint(target, samples):
    """One vote per distinct acquisition timestamp, all in the same odom epoch."""
    unique = {s.get('sample_id',s['stamp_s']): s for s in samples}
    if not unique:
        raise ValueError('No fresh paired localization samples in this cycle')
    epochs = {s['epoch'] for s in unique.values()}
    if len(epochs) != 1:
        raise ValueError('Localization epoch changed inside the cycle')
    points = [map_target_in_odom(target, s['map_pose'], s['odom_pose']) for s in unique.values()]
    return dict(x=math.fsum(p['x'] for p in points)/len(points),
                y=math.fsum(p['y'] for p in points)/len(points))
