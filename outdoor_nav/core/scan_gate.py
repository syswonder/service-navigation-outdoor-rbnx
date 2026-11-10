"""Suppress lidar obstacle marking while the robot uses the stair gait."""

import math


def stairs_obstacle_off(gait_status, enabled=True):
    """True once a stair gait has been requested and is being retried or confirmed.

    The request starts one metre before the stair section, so the treads are
    ignored before the first step rather than after it.
    """
    if not enabled or not isinstance(gait_status, dict):
        return False
    return gait_status.get("target_code") == 0x3003 and (
        gait_status.get("confirmed") or (gait_status.get("attempts") or 0) > 0
    )


def gated_ranges(ranges, suppress):
    """Infinite ranges clear obstacle marks instead of freezing the last tread."""
    return [math.inf] * len(ranges) if suppress else list(ranges)
