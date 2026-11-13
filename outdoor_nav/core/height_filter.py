"""Distance-graded minimum height; removes the rising ground on a ramp."""

import numpy as np


def body_band(near_m, far_m, max_m, lidar_height_m):
    """Convert ground-relative heights to the lidar-centred body frame."""
    return near_m - lidar_height_m, far_m - lidar_height_m, max_m - lidar_height_m


def graded_minimum(distance_sq, near_m, far_m, ramp_m):
    """Minimum accepted height at each horizontal distance in metres."""
    distance = np.sqrt(distance_sq)
    return near_m + (far_m - near_m) * np.minimum(distance / ramp_m, 1.0)


def filter_xyz(x, y, z, near_m, far_m, ramp_m, max_m):
    """Keep finite points between the graded minimum and the fixed ceiling.

    A single low threshold includes the ramp surface far ahead; raising the
    threshold with horizontal distance ignores that surface while keeping
    obstacles near the robot. `far_m` is reached at `ramp_m` and beyond.
    """
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    minimum = graded_minimum(np.where(finite, x * x + y * y, 0.0), near_m, far_m, ramp_m)
    return finite & (z >= minimum) & (z <= max_m)
