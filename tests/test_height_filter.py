import math
import numpy as np
import pytest

from outdoor_nav.core.height_filter import body_band, filter_xyz, graded_minimum
from outdoor_nav.core.scan_gate import gated_ranges, stairs_obstacle_off


def test_ground_relative_band_is_offset_by_measured_lidar_height():
    assert body_band(.2, .5, .7, .42) == pytest.approx((-.22, .08, .28))


def test_graded_minimum_ramps_from_near_to_far():
    d = np.array([0., 4., 8., 20.])
    assert np.allclose(graded_minimum(d * d, .2, .5, 8.), [.2, .35, .5, .5])


def test_ramp_surface_far_away_is_removed_but_near_obstacle_survives():
    # A 4% ramp puts the surface at 0.32 m at 8 m; a knee-high obstacle at 1 m remains.
    x = np.array([1., 8., 8., 1.])
    y = np.zeros(4)
    z = np.array([.25, .32, .60, .10])
    keep = filter_xyz(x, y, z, .2, .5, 8., 1.5)
    assert keep.tolist() == [True, False, True, False]


def test_filter_ceiling_and_non_finite_points():
    x = np.array([1., 1., np.nan])
    y = np.array([0., 0., 0.])
    z = np.array([.3, 2., .3])
    keep = filter_xyz(x, y, z, .2, .5, 8., 1.5)
    assert keep.tolist() == [True, False, False]


def test_stairs_only_suppress_after_a_request():
    assert not stairs_obstacle_off(None)
    assert not stairs_obstacle_off({"target_code": 0x3002, "attempts": 3})
    assert stairs_obstacle_off({"target_code": 0x3003, "attempts": 1})
    assert stairs_obstacle_off({"target_code": 0x3003, "confirmed": True})
    assert not stairs_obstacle_off({"target_code": 0x3003, "attempts": 1}, enabled=False)


def test_suppressed_scan_is_infinite_and_clears():
    assert gated_ranges([1., 2.], True) == [math.inf, math.inf]
    assert gated_ranges([1., 2.], False) == [1., 2.]
