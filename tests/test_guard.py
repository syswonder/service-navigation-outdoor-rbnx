import ast
import math
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist
from outdoor_nav.guard import Watchdog


def fixture():
    command = Twist()
    command.linear.x = 0.2
    return S(
        command=command,
        config={
            "motion_enabled": True,
            "sensor_timeout_s": 1.0,
            "command_timeout_s": 0.5,
            "max_speed_mps": 0.4,
            "max_angular_speed_rps": 0.4,
        },
        conflict=False,
        diagnostic=False,
        ready=True,
        geo_at=10.0,
        scan_at=10.0,
        command_at=10.0,
        pub=Mock(topic_name="/robonix/nomotion/outdoor_cmd_vel"),
        status_pub=Mock(),
    )


@pytest.mark.parametrize(
    "field,value",
    [
        ("conflict", True),
        ("diagnostic", True),
        ("ready", False),
        ("geo_at", 6.9),
        ("scan_at", 8.0),
        ("command_at", 9.0),
    ],
)
def test_failure_is_zero_velocity(field, value):
    n = fixture()
    setattr(n, field, value)
    with patch("outdoor_nav.guard.time.monotonic", return_value=10.0):
        Watchdog.tick(n)
    sent = n.pub.publish.call_args.args[0]
    assert sent.linear.x == 0 and sent.angular.z == 0


def test_guard_limits_and_nonfinite_velocity():
    n = fixture()
    n.command.linear.x = 4.0
    n.command.linear.y = 2.0
    n.command.angular.z = -3.0
    with patch("outdoor_nav.guard.time.monotonic", return_value=10.0):
        Watchdog.tick(n)
    s = n.pub.publish.call_args.args[0]
    assert s.linear.x == 0.4 and s.linear.y == 0 and s.angular.z == -0.4
    n.command.linear.x = -1.0
    with patch("outdoor_nav.guard.time.monotonic", return_value=10.0):
        Watchdog.tick(n)
    assert n.pub.publish.call_args.args[0].linear.x == 0
    n.command.linear.x = math.nan
    with patch("outdoor_nav.guard.time.monotonic", return_value=10.0):
        Watchdog.tick(n)
    assert n.pub.publish.call_args.args[0].linear.x == 0


def test_localization_has_three_seconds_but_command_and_scan_stay_fresh():
    n = fixture()
    n.geo_at = 7.1
    with patch("outdoor_nav.guard.time.monotonic", return_value=10.0):
        Watchdog.tick(n)
    assert n.pub.publish.call_args.args[0].linear.x == .2


def test_owner_conflict_can_clear_without_replaying_a_cached_command():
    n = fixture()
    count = [2]
    n.count_publishers = lambda *_: count[0]
    n.get_logger = lambda: Mock()
    Watchdog.check_owners(n)
    assert n.conflict and n.command_at == 0. and n.command.linear.x == 0.
    n.command.linear.x = .2
    n.command_at = 9.9
    count[0] = 1
    Watchdog.check_owners(n)
    assert not n.conflict and n.command_at == 0. and n.command.linear.x == 0.

@pytest.mark.parametrize("depth_live", [True, False])
def test_ranger_blocks_controller_even_when_original_map_is_free(depth_live):
    import numpy as np
    from outdoor_nav.core.ground_support import CELL, SIZE, SupportMask

    n = fixture()
    n.config.update(ranger_descent_enabled=True, ranger_depth_timeout_s=.35,
                    ranger_brake_mps2=.55)
    n.command.linear.x = .22
    n.original_costmap = np.zeros((SIZE, SIZE), dtype=np.int8)
    n.get_clock = lambda: S(now=lambda: S(nanoseconds=10_000_000_000))
    n.speed_sample = (.22, 10., 10.)
    n.support_tf = Mock()
    support = SupportMask(np.zeros_like(n.original_costmap, dtype=bool), (-8., -8.), CELL, 10., .3)
    valid = SupportMask(np.ones_like(n.original_costmap, dtype=bool), (-8., -8.), CELL, 10., .3)
    feeds = [valid]
    n.support = S(fresh=lambda: feeds[-1])
    n.ranger_permits = lambda command, now: Watchdog.ranger_permits(n, command, now)
    pose = S(transform=S(translation=S(x=0., y=0.), rotation=S(x=0., y=0., z=0., w=1.)))
    with patch("outdoor_nav.guard.time.monotonic", return_value=10.), patch(
        "outdoor_nav.ground_support_node.latest_base", return_value=pose
    ):
        Watchdog.tick(n)
        assert n.pub.publish.call_args.args[0].linear.x == pytest.approx(.22)
        feeds.append(support if depth_live else None)
        for _ in range(10):
            Watchdog.tick(n)
            sent = n.pub.publish.call_args.args[0]
            assert sent.linear.x == 0. and sent.angular.z == 0.
            speed = max(sent.linear.x, n.speed_sample[0] - .55*.05)
            n.speed_sample = (speed, 10., 10.)
    sent = n.pub.publish.call_args.args[0]
    assert n.speed_sample[0] == 0.
    assert n.original_costmap[SIZE//2, SIZE//2] == 0
    assert sent.linear.x == 0 and sent.angular.z == 0
    assert n.command.linear.x == .22
