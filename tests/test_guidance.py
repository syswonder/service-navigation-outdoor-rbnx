import pytest
from outdoor_nav.core.guidance import RouteGuidance
from outdoor_nav.core.tasks import ProgressBudget
from outdoor_nav.core.scan_health import healthy_scan
from types import SimpleNamespace as S
import math


def test_occupied_via_moves_only_to_known_free_space_within_one_meter():
    from outdoor_nav.core.guidance import free_via_point
    grid = S(info=S(width=40, height=40, resolution=.1,
                    origin=S(position=S(x=0., y=0.))), data=[100]*1600)
    point = {"x": 2.05, "y": 2.05}
    grid.data[20*40+27] = 0
    adjusted = free_via_point(point, grid)
    assert math.isclose(adjusted["x"], 2.75)
    assert math.isclose(adjusted["y"], 2.05)
    grid.data[20*40+27] = -1
    grid.data[20*40+35] = 0
    assert free_via_point(point, grid) == point


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


def test_skip_evidence_requires_recent_known_occupied_nearby_cell():
    from outdoor_nav.core.guidance import observed_obstacle
    grid = S(header=S(frame_id='map', stamp=S(sec=10, nanosec=0)),
             info=S(width=10, height=10, resolution=1.,
                    origin=S(position=S(x=0., y=0.),
                             orientation=S(x=0., y=0., z=0., w=1.))), data=[0]*100)
    point, robot = {'x':2., 'y':2.}, {'x':0., 'y':0.}
    for value, expected in [(-1, False), (0, False), (70, False), (99, True), (100, True)]:
        grid.data[22] = value
        assert observed_obstacle(point, grid, robot, 10.1) is expected
    assert not observed_obstacle(point, grid, robot, 13.1)
    assert not observed_obstacle(point, grid, {'x':9., 'y':9.}, 10.1)


def planner_harness():
    # Execute the production async methods without importing ROS on this host.
    import ast
    from pathlib import Path
    from outdoor_nav.core.guidance import validate_point, free_via_point, observed_obstacle
    tree = ast.parse((Path(__file__).parents[1]/'outdoor_nav/executor.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Executor')
    methods = [n for n in cls.body if isinstance(n, ast.AsyncFunctionDef)
               and n.name in ('guided_path', 'plan_guided_window')]
    def pose(x=0., y=0.):
        return S(pose=S(position=S(x=x, y=y), orientation=S(z=0., w=1.)))
    def goal():
        return S(goal=pose(), start=pose())
    scope = dict(Path=lambda:S(header=S(), poses=[]), validate_point=validate_point,
                 free_via_point=free_via_point, observed_obstacle=observed_obstacle,
                 ComputePathToPose=S(Goal=goal), GoalStatus=S(STATUS_SUCCEEDED=4),
                 healthy_scan=healthy_scan, math=math)
    exec(compile(ast.Module(body=methods, type_ignores=[]), '<production>', 'exec'), scope)
    return scope, pose


def test_failed_occupied_bend_skips_only_after_next_path_succeeds():
    import asyncio
    from functools import partial

    scope, pose = planner_harness()
    a, b, c = {'x':0., 'y':0.}, {'x':2., 'y':0.}, {'x':2., 'y':2.}
    def run(final=False, blocked_next=False, stale=False, no_hit=False):
        g = RouteGuidance([a,b] if final else [a,b,c], a, b if final else c)
        grid = S(header=S(frame_id='map', stamp=S(sec=1 if stale else 10, nanosec=0)),
                 info=S(width=10, height=10, resolution=1.,
                        origin=S(position=S(x=0.,y=0.), orientation=S(x=0.,y=0.,z=0.,w=1.))),
                 data=[100]*100)  # No nearby free cell: shifted target also fails.
        called=[]
        def send(req):
            p=req.goal.pose.position;called.append((p.x,p.y))
            failed = len(called)==1 or blocked_next
            result=S(status=6 if failed else 4,
                     result=S(path=S(poses=[] if failed else [pose(0,0),pose(p.x,p.y)])))
            return S(accepted=True,get_result_async=lambda:result,cancel_goal_async=lambda:None)
        async def result(value, task):return value
        n=S(hardware=True,global_map=grid,scan=S(header=S(frame_id='base_link',stamp=S(sec=10,nanosec=0)),
            ranges=[math.inf if no_hit else 2.],range_min=.1,range_max=8.,angle_min=0.,angle_increment=.1),
            config={'sensor_timeout_s':1.}, path_client=S(send_goal_async=send),result=result,
            get_clock=lambda:S(now=lambda:S(nanoseconds=10_100_000_000,to_msg=lambda:None)))
        n.plan_guided_window = partial(scope['plan_guided_window'], n)
        path=asyncio.run(scope['guided_path'](n,g,(1,),{**a,'yaw':0.},'task'))
        return path,g,called
    path,g,called=run()
    assert path is not None and called==[(2.,0.),(2.,2.)] and g.skipped=={1}
    assert g.window(a)[0]==2
    for kwargs in ({'final':True},{'blocked_next':True},{'stale':True},{'no_hit':True}):
        path,g,called=run(**kwargs)
        assert path is None and not g.skipped


def test_obstacle_detour_does_not_require_touching_the_bend():
    reference = [{'x':0.,'y':0.}, {'x':3.,'y':0.}, {'x':3.,'y':2.}, {'x':6.,'y':2.}]
    guidance = RouteGuidance(reference, reference[0], reference[-1])
    for point in ({'x':2.6,'y':.23}, {'x':3.4,'y':.73}, {'x':3.76,'y':1.37},
                  {'x':4.34,'y':1.93}, {'x':5.56,'y':2.}):
        guidance.window(point)
    assert guidance.final_leg
    assert guidance.remaining_length(point) == pytest.approx(.44)


def test_long_route_uses_a_local_horizon_not_the_distant_final_goal():
    start = {'x':0., 'y':0.}
    goal = {'x':107., 'y':47.}
    guidance = RouteGuidance([start, {'x':107., 'y':0.}, goal], start, goal)
    window = guidance.window(start)
    assert not guidance.final_leg
    assert all(math.hypot(guidance.points[i]['x'], guidance.points[i]['y']) <= 6.01 for i in window)
    assert guidance.remaining_length(start) == pytest.approx(154.)


def test_guided_path_bypasses_straight_samples_and_retries_when_needed():
    import asyncio
    from functools import partial

    scope, pose = planner_harness()
    start, end = {'x': 0., 'y': 0.}, {'x': 6., 'y': 0.}
    guidance = RouteGuidance([], start, end)
    calls = []

    def run(failed_calls, short_station=False):
        calls.clear()

        def send(request):
            point = request.goal.pose.position
            calls.append(point.x)
            success = len(calls) not in failed_calls
            end_x = -0.15 if short_station and point.x == 2. else point.x
            result = S(status=4 if success else 6, result=S(path=S(
                poses=[request.start, pose(end_x, point.y)] if success else []
            )))
            return S(accepted=True, get_result_async=lambda: result)

        async def result(value, task):
            return value

        node = S(
            hardware=False, config={}, global_map=None, scan=None, result=result,
            path_client=S(send_goal_async=send),
            get_clock=lambda: S(now=lambda: S(to_msg=lambda: None, nanoseconds=0)),
        )
        node.plan_guided_window = partial(scope['plan_guided_window'], node)
        return asyncio.run(scope['guided_path'](node, guidance, (1, 2, 3), start, 'task'))

    assert run(set()).poses[-1].pose.position.x == 6.
    assert calls == [6.]
    assert run({1, 4}).poses[-1].pose.position.x == 4.
    assert calls == [6., 2., 4., 6.]
    assert guidance.points[-1] == end
    assert not guidance.skipped
    assert run({1, 3}, short_station=True) is None
    assert calls == [6., 2., 4.]
    assert run({1}, short_station=True).poses[-1].pose.position.x == 6.
    assert calls == [6., 2., 4., 6.]


def test_progress_advances_at_the_adjusted_station():
    start, goal = {'x': 0., 'y': 0.}, {'x': 6., 'y': 0.}
    guidance = RouteGuidance([], start, goal)
    position = {'x': 1.75, 'y': 0.8}
    assert guidance.window(position)[0] == 1

    guidance.adjusted[1] = {'x': 2., 'y': 0.8}
    assert guidance.window(position)[0] == 2
    assert guidance.points[1] == {'x': 2., 'y': 0.}
    assert guidance.points[-1] == goal
