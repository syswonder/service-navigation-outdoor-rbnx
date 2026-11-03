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
        ("geo_at", 8.0),
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


def test_guard_logic_matches_original():
    root = Path(__file__).parents[1]

    def methods(path):
        c = next(
            n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef)
        )
        return {n.name: ast.dump(n) for n in c.body if isinstance(n, ast.FunctionDef)}

    old, new = (
        methods(root / "tests/upstream_guard.py.txt"),
        methods(root / "outdoor_nav/guard.py"),
    )
    for name in ("check_owners", "receive", "geo", "scan", "tick", "destroy_node"):
        assert old[name] == new[name], name
