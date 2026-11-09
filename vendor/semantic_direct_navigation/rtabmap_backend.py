"""Bounded RTAB-Map request pairing and common calibrated height policy.

ROS transport is in depth_node. This module does not publish or command motion.
"""
import numpy as np


class FrameSlot:
    """One in-flight frame; exact nanosecond pairing, no backlog/re-stamping."""
    def __init__(self, timeout=.2):
        if not 0 < timeout <= .3:
            raise ValueError('Invalid RTAB-Map deadline')
        self.timeout = timeout
        self.pending = None
        self.last_key = -1
        self.skipped = 0

    def begin(self, key, now, payload):
        if self.pending is not None or key <= self.last_key:
            self.skipped += 1
            return False
        self.last_key = key
        self.pending = dict(key=key, started=now, payload=payload, outputs={})
        return True

    def expired(self, now):
        if self.pending is not None and now-self.pending['started'] >= self.timeout:
            old, self.pending = self.pending, None
            return old
        return None

    def add(self, key, kind, points, now):
        p = self.pending
        if (p is None or key != p['key'] or kind not in ('ground', 'obstacles')
                or now-p['started'] >= self.timeout):
            return None
        p['outputs'][kind] = points
        if len(p['outputs']) == 2:
            self.pending = None
            return p
        return None


def fuse_rtabmap(ground, obstacles, raw_hazards, reference_status,
                 expected_ground_z=-.18, min_height=.10):
    """Positive RTAB clusters >=10 cm; keep previously measured drop hazards.

    The old fitter supplies a reference plane, not positive obstacle labels.
    Without its reliable fit, calibrated nominal height is explicit and only
    close-to-plane RTAB ground grants coverage. No ground is manufactured.
    """
    trusted = bool(reference_status.get('valid'))
    plane = np.asarray(reference_status['plane_abc'] if trusted
                       else [0., 0., expected_ground_z], float)
    if (plane.shape != (3,) or not np.isfinite(plane).all()
            or not np.isfinite(min_height) or min_height != .10):
        raise ValueError('Invalid height policy')

    def clean(points):
        a = np.asarray(points, float).reshape(-1, 3)
        return a[np.isfinite(a).all(axis=1) & (a[:, 0] > -.8)
                 & (a[:, 0] < 5.) & (abs(a[:, 1]) < 3.)]

    def height(a):
        return a[:, 2] - (a[:, :2] @ plane[:2] + plane[2])

    g, o = clean(ground), clean(obstacles)
    if len(g) + len(o) == 0:
        raise ValueError('RTABMAP_NO_CLASSIFIED_GEOMETRY')
    h = height(o)
    positive = o[(h >= min_height) & (h < 1.5)]
    g = g[abs(height(g)) <= .03]
    raw = clean(raw_hazards)
    drops = raw[height(raw) < -.10] if trusted else np.empty((0, 3))
    combined = np.vstack((positive, drops))
    return g, combined, dict(valid=True, reason='OK', source='RTABMAP_GEOMETRY',
        plane_abc=plane.tolist(), plane_source='CURRENT_DEPTH_FIT' if trusted else 'CALIBRATED_NOMINAL',
        height_threshold_m=min_height, positive_obstacle_points=len(positive),
        retained_drop_points=len(drops), ground_points=len(g), obstacle_points=len(combined),
        reference_fit_reason=reference_status.get('reason', 'UNKNOWN'))
