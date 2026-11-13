"""ROS-side transition hooks, without starting ROS or publishing motion."""

import asyncio
from types import SimpleNamespace as S
from unittest.mock import Mock, AsyncMock, patch
import pytest

pytest.importorskip("rclpy")
from outdoor_nav.runtime import OutdoorExecutor
from outdoor_nav.executor import Executor


def fixture():
    return S(
        geo={"grade": "fixed", "ready": True, "fusion": {"recovery_id": 2}},
        handled_recovery=1,
        stopping=False,
        command_pub=Mock(),
        tasks=S(update=Mock(), snapshot=Mock(return_value={})),
        active_follow_requests=["old"],
        active_follow_results={},
        release_controller=AsyncMock(return_value=True),
        confirm_stop=AsyncMock(return_value=True),
        gait_tick=Mock(),
        config={"navigation_recovery_timeout_s": 0.1},
        interrupted=Mock(return_value=False),
        pause=AsyncMock(),
    )


def test_recovery_releases_controller_before_resuming():
    n = object.__new__(OutdoorExecutor)
    n.__dict__.update(vars(fixture()))
    with patch.object(
        Executor, "wait_for_ready_pose", new=AsyncMock(return_value={"x": 0, "y": 0})
    ):
        asyncio.run(OutdoorExecutor.wait_for_ready_pose(n, "task"))
    n.release_controller.assert_awaited_once_with(["old"], {})
    n.confirm_stop.assert_awaited_once()
    assert n.handled_recovery == 2 and n.stopping


def test_arrival_waits_for_rtk_and_handles_cancel():
    n = fixture()
    n.geo.update(grade="inertial")

    async def restore(*_):
        n.geo["grade"] = "float"

    n.pause = restore
    asyncio.run(OutdoorExecutor.wait_for_arrival_confirmation(n, "task"))
    assert n.stopping
    n.geo["grade"] = "inertial"
    n.interrupted.return_value = True
    with pytest.raises(InterruptedError):
        asyncio.run(OutdoorExecutor.wait_for_arrival_confirmation(n, "task"))


def test_stop_failure_recovers_only_after_release_and_standstill():
    n = S(stop_failure='unverified stop', stop_recovery=([], {}), owner=None,
          stop_recovering=False, clock_rewound=False, startup_stopped=False,
          release_controller=AsyncMock(return_value=False),
          confirm_stop=AsyncMock(return_value=True))
    asyncio.run(OutdoorExecutor.recover_stop(n))
    assert n.stop_failure and not n.startup_stopped
    n.confirm_stop.assert_not_awaited()
    n.release_controller.return_value = True
    n.confirm_stop.return_value = False
    asyncio.run(OutdoorExecutor.recover_stop(n))
    assert n.stop_failure
    n.confirm_stop.return_value = True
    asyncio.run(OutdoorExecutor.recover_stop(n))
    assert n.stop_failure is None and n.stop_recovery is None and n.startup_stopped
