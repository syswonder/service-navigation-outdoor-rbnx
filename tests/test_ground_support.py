import math

import numpy as np

from outdoor_nav.core.descent import Survey
from outdoor_nav.core.ground_support import CELL, SIZE, GroundGrid, SupportMask


def terrain(drop=False, hole=False):
    x, y = np.meshgrid(np.arange(-1., 2., .04), np.arange(-1., 1.1, .04))
    z = np.minimum(-.16*x, 0.)
    if drop:
        z[x >= .70] -= .25
    keep = (x < .5) | (x > .85) if hole else np.ones_like(x, dtype=bool)
    return np.column_stack((x[keep], y[keep], z[keep]))


def observed(points):
    grid = GroundGrid(.075, math.tan(math.radians(15)), .24)
    cells = grid.update((0., 0.), 0., points, 1.)
    return grid, SupportMask(cells, grid.origin, CELL, 1., .24)


def test_confirmed_ramp_can_be_traversed_but_drop_and_hole_cannot():
    _, flat = observed(terrain())
    _, drop = observed(terrain(drop=True))
    _, hole = observed(terrain(hole=True))
    assert flat.corridor((0., 0.), [(1.2, 0.)])
    assert not drop.corridor((0., 0.), [(1.2, 0.)])
    assert not hole.corridor((0., 0.), [(1.2, 0.)])
    assert not flat.clear([(100., 0.)])


def test_measurement_expiry_does_not_clear_an_unobserved_corridor():
    grid, _ = observed(terrain())
    cells = grid.update((0., 0.), 0., np.empty((0, 3)), 47.)
    support = SupportMask(cells, grid.origin, CELL, 47., .24)
    assert support.clear([(0., 0.)])
    assert not support.corridor((0., 0.), [(1.2, 0.)])


def test_rejects_old_free_path_at_the_actual_braking_horizon():
    cells = np.zeros((SIZE, SIZE), dtype=bool)
    mid = SIZE // 2
    cells[mid-2:mid+3, mid-2:mid+3] = True
    support = SupportMask(cells, (-(mid+.5)*CELL,)*2, CELL, 1., .24)
    config = {"ranger_depth_timeout_s": .35, "ranger_brake_mps2": .55}
    assert support.clear([(0., 0.)])
    assert not support.permits((0., 0.), 0., .22, 0., .22, config)


def test_survey_uses_support_and_skips_failed_target():
    _, support = observed(terrain())
    survey = Survey()
    target = survey.candidates(support, (0., 0.), (1.5, 0.))[0]
    assert target == (1.5, 0.)
    for _ in range(3):
        survey.failed(target, CELL)
    assert target not in survey.candidates(support, (0., 0.), (1.5, 0.))
