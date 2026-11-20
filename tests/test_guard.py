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
