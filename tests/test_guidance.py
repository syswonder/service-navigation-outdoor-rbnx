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


def test_long_approach_keeps_short_horizon_and_entry_turn_without_expanding_route():
    import tracemalloc

    start, entry, goal = {"x": -10000000.0, "y": 0.0}, {"x": 0.0, "y": 0.0}, {"x": 0.0, "y": 20.0}
    tracemalloc.start()
    try:
        guidance = RouteGuidance([start, entry, goal], start, goal)
        assert guidance.remaining_length(start) == 10000020.0
        window = guidance.window(start)
        assert all(distance_from_start <= 6.0 for distance_from_start in (
            math.dist(tuple(start.values()), tuple(guidance.points[i].values())) for i in window))
        peak = tracemalloc.get_traced_memory()[1]
    finally:
        tracemalloc.stop()
    assert peak < 1024 * 1024
    guidance.index = 4999999
    position = {"x": -2.0, "y": 0.0}
    assert guidance.remaining_length(position) == 22.0
    assert guidance.window(position)[0] == 5000000
    assert guidance.points[5000000] == entry
    assert 5000000 in guidance.bends
    guidance.index = len(guidance.points) - 1
    assert guidance.final_leg and guidance.remaining_length(goal) == 0.0


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
