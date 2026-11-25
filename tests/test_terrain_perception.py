from collections import deque
from types import SimpleNamespace as S

import pytest

pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformException
from outdoor_nav.terrain_perception import TerrainPerception


def perception():
    node = object.__new__(TerrainPerception)
    labels, depth, di, ci = Image(), Image(), CameraInfo(), CameraInfo()
    for message in (labels, depth, di, ci):
        message.header.stamp = Time(sec=10)
        message.header.frame_id = 'camera'
    node.last_stamp = node.last_now = 0.
    node.p = {'observation_timeout_s': 1.}
    node.pending = deque([(10., (labels, depth, di, ci))], maxlen=30)
    node.get_clock = lambda: S(now=lambda: S(nanoseconds=10_200_000_000))
    return node


def test_delayed_capture_time_tf_leaves_frame_pending_without_restamping():
    node = perception()
    def missing(*args):
        raise TransformException('not caught up')
    node.tf = S(lookup_transform=missing)
    node.process = lambda *args: pytest.fail('must wait for actual capture-time TF')
    node.process_ready()
    assert len(node.pending) == 1
    assert node.pending[0][1][1].header.stamp.sec == 10


def test_four_input_pair_uses_depth_stamp_after_confidence_stream_removal():
    node = perception()
    stamps = []
    def transform(target, source, stamp):
        stamps.append(stamp.nanoseconds)
        return S()
    called = []
    node.tf = S(lookup_transform=transform)
    node.process = lambda *args: called.append(args)
    node.process_ready()
    assert stamps == [10_000_000_000, 10_000_000_000]
    assert len(called) == 1 and len(called[0][1]) == 4
