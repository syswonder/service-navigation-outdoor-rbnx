import math
from types import SimpleNamespace as S
import pytest
from outdoor_nav.core.guidance import RouteGuidance, gait_boundary_points
from outdoor_nav.core.gait import GaitSchedule, FLAT, STAIRS
from outdoor_nav.core.scan_health import acquisition_monotonic

REF = [{'x':0.,'y':0.},{'x':3.,'y':0.},{'x':3.,'y':3.},{'x':6.,'y':3.}]
INT = [dict(start_m=0.,end_m=3.,gait='flat',gait_code=FLAT),
       dict(start_m=3.,end_m=6.,gait='stairs',gait_code=STAIRS),
       dict(start_m=6.,end_m=9.,gait='flat',gait_code=FLAT)]


def test_gait_corner_cannot_be_cut_at_normal_waypoint_tolerance():
    checkpoints=gait_boundary_points(REF,INT)
    g=RouteGuidance(REF,REF[0],REF[-1],checkpoints=checkpoints,checkpoint_tolerance=.25)
    g.window({'x':2.5,'y':0.})
    boundary=g.points.indices[1]
    assert g.window({'x':2.6,'y':.1})[-1] == boundary
    assert g.window({'x':2.6,'y':.7})[-1] == boundary
    assert g.window({'x':2.85,'y':.1})[0] > boundary


def test_straight_gait_boundary_survives_simplification_and_window():
    reference=[{'x':0.,'y':0.},{'x':9.,'y':0.}]
    g=RouteGuidance(reference,reference[0],reference[-1],
                    checkpoints=gait_boundary_points(reference,INT))
    end=g.window(reference[0])[-1]
    assert g.points[end] == {'x':3.,'y':0.}
    assert end in g.checkpoints


def test_gait_progress_does_not_skip_an_unvisited_boundary():
    g=GaitSchedule(REF,INT)
    g.route.update({'x':2.6,'y':.7})
    assert g.route.index == 0 and g.route.station < 3.
    g.route.update({'x':2.9,'y':.05})
    assert g.route.index == 1


def test_late_lidar_receipt_keeps_its_original_age():
    scan=S(header=S(stamp=S(sec=9,nanosec=200000000)))
    assert acquisition_monotonic(scan,10.,100.) == pytest.approx(99.2)


def test_task_tolerance_does_not_replace_controller_tolerance(tmp_path):
    from pathlib import Path
    import yaml
    from outdoor_nav.configuration import validate
    p=Path(__file__).resolve().parents[1]/'config/nav2_params.example.yml'
    doc=yaml.safe_load(p.read_text())
    doc['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']=.18
    f=tmp_path/'nav2.yaml';f.write_text(yaml.safe_dump(doc))
    config=validate({'params_file':str(f),'policy':{'fixed_arrival_m':3.,'float_arrival_m':3.}})
    assert config['policy']['controller_goal_tolerance_m'] == .18
    assert config['policy']['fixed_arrival_m'] == 3.


def test_standing_mode_can_prepare_but_wrong_mode_cannot():
    from outdoor_nav.core.gait import chassis_prepared
    body=dict(state=17,usage_mode=1,gait=4097,gait_output_enabled=True,
              motion_feedback_age_s=.01,usage_mode_age_s=.01)
    assert chassis_prepared(body,.01)
    assert not chassis_prepared(body|{'usage_mode':0},.01)
    assert not chassis_prepared(body,2.)


def test_journal_from_another_version_still_loads(tmp_path):
    import json
    from outdoor_nav.core.tasks import TaskStore
    journal = tmp_path / 'tasks.json'
    journal.write_text(json.dumps({
        'current_id': 'a' * 32,
        'tasks': [{'task_id': 'a' * 32, 'goal': {'x': 0., 'y': 0.}, 'blocked_road_ids': [],
                   'state': 'navigating',
                   'terrain_status': {'phase': 'stairs'}, 'vision_epoch': 3}],
    }))
    store = TaskStore(journal=journal)
    saved = store.snapshot('a' * 32)
    assert saved['state'] == 'failed'
    assert 'terrain_status' not in saved


def test_checkpoint_leg_gets_a_tighter_goal_tolerance():
    from outdoor_nav.core.guidance import leg_goal_tolerance
    # The window ends at a gait boundary: the controller must still drive there.
    assert leg_goal_tolerance(True, .5, .25) == .125
    assert leg_goal_tolerance(True, .1, .25) == .1
    # Ordinary legs keep the configured task tolerance.
    assert leg_goal_tolerance(False, .5, .25) == .5
