from outdoor_nav.core.guidance import RouteGuidance
from outdoor_nav.core.tasks import ProgressBudget
from outdoor_nav.core.scan_health import healthy_scan
from types import SimpleNamespace as S
import math


def test_ordered_u_route_and_bounded_horizon():
    a = {"x": 0.0, "y": 0.0}
    b = {"x": 0.0, "y": 0.1}
    g = RouteGuidance([a, {"x": 10.0, "y": 0.0}, {"x": 10.0, "y": 0.1}, b], a, b)
    assert (
        sum(
            math.dist(tuple(x.values()), tuple(y.values()))
            for x, y in zip(g.points, g.points[1:])
        )
        > 20
    )
    window = g.window(a)
    assert window[-1] < len(g.points) - 1
    assert math.dist(tuple(a.values()), tuple(g.points[window[-1]].values())) <= 6.0


def test_scan_inf_semantics_and_staleness():
    s = S(
        header=S(stamp=S(sec=1, nanosec=0), frame_id="base_link"),
        ranges=[math.inf],
        range_min=0.5,
        range_max=8.0,
        angle_increment=0.01,
    )
    assert healthy_scan(s, 1.1, 1.0)
    assert not healthy_scan(s, 2.1, 1.0)
    for ranges in ([], [math.nan], [-math.inf], [0.0]):
        s.ranges = ranges
        assert not healthy_scan(s, 1.1, 1.0)


def test_same_failure_and_progress_budgets():
    b = ProgressBudget(planner_patience_s=60.0, controller_patience_s=60.0)
    assert b.planned(False, 10.0) and b.planned(False, 69.0)
    assert not b.planned(False, 70.0)
    b.moved(0.0, 0.0, 100.0)
    for _ in range(20):
        assert b.controller_ended(120.0)
    assert not b.controller_ended(161.0)
