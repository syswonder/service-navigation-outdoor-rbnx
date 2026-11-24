"""Use measured-time lidar/TF pairs; never restamp an observation to fit TF."""
from collections import deque
from threading import Lock
from types import SimpleNamespace as S

import pytest

pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from outdoor_nav.cycle_runtime import CycleNavigation, SnapshotUnavailable


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
    c.samples = deque()
    c.camera = None
    c.input_lock = Lock()
    c.p = dict(period_s=2., grid_resolution_m=.5, grid_half_width_m=2.)

    def transform(target, source, stamp):
        if stamp.nanoseconds > 10_000_000_000:
            raise TransformException('TF has not caught up')
        return S(transform=S(rotation=S(x=0., y=0., z=0., w=1.), translation=S(x=0., y=0.)))

    c.n = S(config={'lidar_enabled': True, 'camera_enabled': False, 'sensor_timeout_s': 2.,
                    'localization_timeout_s': 3.},
            tf=S(lookup_transform=transform),
            get_clock=lambda: S(now=lambda: S(nanoseconds=10_200_000_000,
                                             to_msg=lambda: Time(sec=10, nanosec=200_000_000))))
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
    with pytest.raises(SnapshotUnavailable, match='acquisition-time TF'):
        c.grid({'x': 0., 'y': 0.}, 10.2)


def test_input_arrival_during_cycle_does_not_change_its_snapshot():
    earlier, later = scan(9, 800_000_000), scan(9, 900_000_000)
    c = cycle([earlier])
    c.scan = earlier
    c.samples.append({'stamp_s': 9.8})
    cutoff, samples, observations = c.snapshot_inputs()
    c.on_scan(later)
    c.samples.append({'stamp_s': 9.9})
    stamp, _ = c.grid({'x': 0., 'y': 0.}, cutoff, observations=observations)
    assert stamp == pytest.approx(9.8)
    assert samples == [{'stamp_s': 9.8}]
    assert c.scan is later


def test_climb_transition_does_not_reuse_a_scan_from_the_previous_mode():
    c = cycle([scan(9, 900_000_000), scan(10, 100_000_000)])
    with pytest.raises(ValueError, match='acquisition-time TF'):
        c.grid({'x': 0., 'y': 0.}, 10.2, since=10.)


def test_late_scan_does_not_replace_newer_acquisition():
    latest = scan(10)
    c = cycle([latest])
    c.scan = latest
    c.on_scan(scan(9, 950_000_000))
    assert c.scan is latest and list(c.scans) == [latest]
