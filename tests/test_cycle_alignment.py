"""Use measured-time lidar/TF pairs; never restamp an observation to fit TF."""
from collections import deque
from types import SimpleNamespace as S

import pytest

pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from outdoor_nav.cycle_runtime import CycleNavigation


def scan(sec, nanosec=0):
    msg = LaserScan()
    msg.header.frame_id = 'base_link'
    msg.header.stamp = Time(sec=sec, nanosec=nanosec)
    msg.angle_increment = .1
    msg.range_min, msg.range_max = .1, 8.
    msg.ranges = [float('inf')]
    return msg


def cycle(values):
    c = object.__new__(CycleNavigation)
    c.scans = deque(values, maxlen=64)
    c.camera = None
    c.p = dict(period_s=2., grid_resolution_m=.5, grid_half_width_m=2.)

    def transform(target, source, stamp):
        if stamp.nanoseconds > 10_000_000_000:
            raise TransformException('TF has not caught up')
        return S(transform=S(rotation=S(x=0., y=0., z=0., w=1.), translation=S(x=0., y=0.)))

    c.n = S(config={'lidar_enabled': True, 'camera_enabled': False}, tf=S(lookup_transform=transform),
            get_clock=lambda: S(now=lambda: S(to_msg=lambda: Time(sec=10, nanosec=200_000_000))))
    return c


def test_uses_fresh_older_scan_without_waiting_or_changing_its_stamp():
    earlier, future = scan(9, 900_000_000), scan(10, 100_000_000)
    c = cycle([earlier, future])
    stamp, grid = c.grid({'x': 0., 'y': 0.}, 10.2)
    assert stamp == pytest.approx(9.9)
    assert earlier.header.stamp.nanosec == 900_000_000
    assert future.header.stamp.nanosec == 100_000_000
    assert grid.header.frame_id == 'odom'
    assert 0 in grid.data


@pytest.mark.parametrize('values', [[], [scan(7)], [scan(10, 100_000_000)]])
def test_no_recent_aligned_scan_is_unknown_not_free(values):
    c = cycle(values)
    with pytest.raises(ValueError, match='acquisition-time TF'):
        c.grid({'x': 0., 'y': 0.}, 10.2)
