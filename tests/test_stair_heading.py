import math
from copy import deepcopy

import numpy as np
import pytest

from outdoor_nav.core.terrain_geometry import stair_heading
from outdoor_nav.core.terrain import TerrainMachine
from outdoor_nav.terrain_config import DEFAULTS
from outdoor_nav.segmentation import predict_labels, UNKNOWN


def flight(yaw, width=1.5, length=2.0, omit_bottom=False):
    x, y = np.meshgrid(np.linspace(.8, .8 + length, 45), np.linspace(-width / 2, width / 2, 35))
    z = -.42 + np.floor((x - .5) / .25) * .15
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.column_stack((c * x.ravel() - s * y.ravel(), s * x.ravel() + c * y.ravel(), z.ravel()))
    return p[x.ravel() > 1.5] if omit_bottom else p


@pytest.mark.parametrize('yaw', [-1.1, -.4, 0., .7, 1.1])
@pytest.mark.parametrize('width,length', [(1., 3.), (3., 1.5)])
@pytest.mark.parametrize('omit_bottom', [False, True])
def test_whole_flight_heading_without_bottom_step(yaw, width, length, omit_bottom):
    result = stair_heading(flight(yaw, width, length, omit_bottom), DEFAULTS | {'ground_z_m': -.42})
    assert result['direction'] == 'up'
    assert result['heading_yaw'] == pytest.approx(yaw, abs=.06)


def test_downward_flight_does_not_resolve_as_upstairs():
    p = flight(0.)
    p[:, 2] = -.42 - (p[:, 2] + .42)
    r = stair_heading(p, DEFAULTS | {'ground_z_m': -.42})
    assert r['direction'] == 'unknown'
    assert r['heading_yaw'] is None


def test_vertical_wall_and_flat_landing_have_no_climb_heading():
    p = flight(0.)
    p[:, 2] = 0.
    assert stair_heading(p, DEFAULTS | {'ground_z_m': -.42})['heading_yaw'] is None
    p[:, [0, 2]] = p[:, [2, 0]]
    p[:, 0] = 1.
    assert stair_heading(p, DEFAULTS | {'ground_z_m': -.42})['heading_yaw'] is None


def test_confidence_filter_stays_inside_model_pipeline():
    class Backend:
        def predict(self, rgb):
            return np.full((2, 2), 53, np.uint16), np.array([[.9, .1], [.8, .2]], np.float32)
    cfg = {'classes': {'53': 'stairs'}, 'road_classes': [], 'confidence_threshold': .55}
    labels, _, _ = predict_labels(Backend(), np.zeros((2, 2, 3), np.uint8), cfg)
    assert labels.tolist() == [[53, UNKNOWN], [53, UNKNOWN]]


def machine():
    p = deepcopy(DEFAULTS)
    p.update(ground_z_m=-.42, enabled=True, can_climb_stairs=True, flat_gait_code=12290, upstairs_gait_code=12291)
    return TerrainMachine(p)


def test_close_stairs_stop_approach_until_heading_resolves():
    m = machine()
    obs = dict(valid=True, stairs=True, direction='up', heading_yaw=None, front_m=.8, stamp=1.)
    out = m.tick(1., enable=True, pitch=0., observation=obs)
    assert not out['motion_allowed'] and out['reason'] == 'stair_stop_range'
    obs['heading_yaw'] = 0.
    out = m.tick(1.1, enable=True, pitch=0., observation=obs,
                 yaw=.5, actual_gait=12290, gait_fresh=True, stopped=True)
    assert out['phase'] == 'align' and not out['motion_allowed']


def test_imu_climb_hold_still_requires_real_gait_feedback():
    m = machine()
    m.state, m.phase, m.reason = 'stairs_up', 'climb', 'climbing'
    kwargs = dict(enable=True, pitch=.3, observation=None, yaw=0., actual_gait=12291)
    assert m.tick(50., gait_fresh=True, **kwargs)['motion_allowed']
    assert not m.tick(51., gait_fresh=False, **kwargs)['motion_allowed']
    m.reset()
    assert m.stair_seen_since == -math.inf
