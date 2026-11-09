"""Shared scan health: a fresh, well-formed empty scene is still a live sensor."""

import math


def healthy_scan(scan, now_s, timeout_s, allow_filtered_empty=False):
    age = now_s - (scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9)
    if not (-0.1 <= age < timeout_s and scan.header.frame_id and scan.ranges):
        return False
    if not (
        math.isfinite(scan.range_min)
        and math.isfinite(scan.range_max)
        and 0 <= scan.range_min < scan.range_max
        and math.isfinite(scan.angle_increment)
        and scan.angle_increment > 0
    ):
        return False
    # +inf means no return within range, not a dead sensor. Empty arrays,
    # all-NaN/-inf/zero scans and stale timestamps must not refresh health.
    return allow_filtered_empty or any(
        r == math.inf or (math.isfinite(r) and scan.range_min <= r <= scan.range_max)
        for r in scan.ranges
    )
