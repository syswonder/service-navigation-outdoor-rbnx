"""Planner choice must not change chassis preparation or feedback requirements."""
import json
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock

import pytest

from outdoor_nav.configuration import validate
from outdoor_nav.core.gait import FLAT, GaitSchedule, chassis_prepared

ROOT = Path(__file__).resolve().parents[1]


def body(**changes):
    return dict(state=17, usage_mode=1, gait=0x1001, ready=False,
                gait_output_enabled=True, motion_feedback_age_s=.01,
                usage_mode_age_s=.01, **changes)


def test_standing_chassis_can_prepare_but_cannot_move_before_confirmation():
    b = body()
    assert chassis_prepared(b, .02)
    schedule = GaitSchedule([{'x': 0., 'y': 0.}, {'x': 2., 'y': 0.}], [], flat_only=True)
    command, status = schedule.tick({'x': 0., 'y': 0.}, 0., b['gait'], .01,
                                    True, stopped=True)
    assert command == FLAT and not status['motion_allowed']
    _, status = schedule.tick({'x': 0., 'y': 0.}, 1., FLAT, .01, True, stopped=True)
    assert status['motion_allowed']


@pytest.mark.parametrize('field,value', [('state', 0), ('usage_mode', 0),
    ('gait_output_enabled', False), ('motion_feedback_age_s', 2.),
    ('usage_mode_age_s', None)])
def test_preparation_never_bypasses_mode_or_feedback(field, value):
    b = body()
    b[field] = value
    assert not chassis_prepared(b, .01)
    assert not chassis_prepared(body(), 2.)


@pytest.mark.parametrize('cycle', [False, True])
@pytest.mark.parametrize('terrain', [False, True])
def test_planner_and_terrain_switches_are_independent(tmp_path, cycle, terrain):
    ground = tmp_path/'ground.local.json'
    ground.write_text(json.dumps({'ground_z_m': -.27}))
    runtime = validate(dict(params_file=str(ROOT/'config/nav2_params.example.yml'),
                            ground_height_file=str(ground),
                            terrain={'enabled': terrain}, decision_cycle={'enabled': cycle}))
    assert runtime['decision_cycle']['enabled'] == cycle
    assert runtime['terrain']['enabled'] == terrain
    assert runtime['policy']['sensor_timeout_s'] == 1.
    assert runtime['policy']['lidar_height_m'] == .27
    assert runtime['policy']['pose_timeout_s'] == runtime['policy']['localization_timeout_s']


@pytest.mark.parametrize('cycle', [False, True])
@pytest.mark.parametrize('terrain', [False, True])
def test_common_gait_handshake_runs_in_each_execution_mode(monkeypatch, cycle, terrain):
    pytest.importorskip('rclpy')
    from outdoor_nav.runtime import OutdoorExecutor
    from outdoor_nav.core.tasks import TaskStore
    from threading import RLock
    n = object.__new__(OutdoorExecutor)
    n.state_lock = RLock()
    n.tasks = TaskStore()
    n.owner = n.tasks.submit({'x': 2., 'y': 0.})['task_id']
    n.tasks.update(n.owner, state='planning', reference_path=[{'x': 0., 'y': 0.}, {'x': 2., 'y': 0.}],
                   intervals=[{'start_m': 0., 'end_m': 2., 'gait': 'flat', 'gait_code': 0x1001}])
    n.runtime = {'gait_switching_enabled': True}
    n.config = dict(sensor_timeout_s=1., stop_hold_s=1., gait_switch_timeout_s=5.)
    n.cycle = S() if cycle else None
    n.terrain = S(inputs=S(p={'can_climb_stairs': False}), tick=Mock()) if terrain else None
    n.gait_task = n.gait_schedule = None
    n.gait_pub = Mock()
    n.command_pub = Mock()
    n.pose = lambda: {'x': 0., 'y': 0., 'age_s': .01}
    n.chassis = body()
    n.chassis_at = n.velocity_at = n.raw_received = 100.
    n.stationary_since = 98.
    n.raw_command = S(linear=S(x=0.), angular=S(z=0.))
    n.status = lambda *_: {'health': {'chassis': True, 'localization': True}}
    monkeypatch.setattr('outdoor_nav.runtime.time.monotonic', lambda: 100.1)
    n.gait_tick()
    assert n.gait_pub.publish.call_args.args[0].data == FLAT
    assert not n.gait_confirmed(n.tasks.snapshot())
    n.chassis['gait'] = FLAT
    n.gait_tick()
    assert n.gait_confirmed(n.tasks.snapshot())
    n.chassis['usage_mode'] = 0
    assert not n.gait_confirmed(n.tasks.snapshot())


def test_explicit_projection_band_is_not_silently_ignored(tmp_path):
    ground = tmp_path/'ground.local.json'
    ground.write_text('{"ground_z_m": -0.27}')
    runtime = validate(dict(params_file=str(ROOT/'config/nav2_params.example.yml'),
        ground_height_file=str(ground), scan_projection={'min_height_m': -.2, 'max_height_m': .4}))
    assert runtime['scan_projection']['fixed_height']
    assert runtime['scan_projection']['min_height_m'] == -.2
    assert runtime['scan_projection']['max_height_m'] == .4
