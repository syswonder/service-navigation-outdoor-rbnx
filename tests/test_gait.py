import math
from types import SimpleNamespace
import threading

import pytest

from outdoor_nav.core.gait import GaitSchedule, RouteStation, validate_intervals, FLAT, STAIRS
from outdoor_nav.core.tasks import TaskStore
from outdoor_nav.requests import submit

REFERENCE = [{"x":0., "y":0.}, {"x":20., "y":0.}]
INTERVALS = [
    {"start_m":0., "end_m":5., "gait":"flat", "gait_code":0x1001},
    {"start_m":5., "end_m":10., "gait":"stairs", "gait_code":0x1003},
    {"start_m":10., "end_m":20., "gait":"flat", "gait_code":FLAT},
]
ORIGIN = {"latitude":30., "longitude":120.}


def tick(schedule, station, now=0., actual=FLAT, age=0., eligible=True):
    return schedule.tick({"x":station, "y":0.}, now, actual, age, eligible)


def test_one_meter_before_and_after_without_stopping():
    schedule = GaitSchedule(REFERENCE, INTERVALS)
    assert tick(schedule, 3.99)[1]["target_code"] == FLAT
    assert tick(schedule, 4.)[0] == STAIRS
    assert tick(schedule, 10.99, 1., STAIRS)[1]["confirmed"]
    assert tick(schedule, 11., 2., STAIRS)[0] == FLAT


def test_retry_is_rate_limited_unbounded_and_requires_real_feedback():
    schedule = GaitSchedule(REFERENCE, INTERVALS)
    assert tick(schedule, 5., 0.)[0] == STAIRS
    assert tick(schedule, 5., .5)[0] is None
    assert tick(schedule, 5., 1.)[1]["attempts"] == 2
    assert tick(schedule, 5., 10000.)[1]["attempts"] == 3
    assert tick(schedule, 5., 10001., STAIRS)[1]["confirmed"]
    # A later posture command resetting flat must be detected, not a permanent latch.
    assert tick(schedule, 5., 10002., FLAT)[0] == STAIRS
    assert not tick(schedule, 5., 10003., STAIRS, age=2.)[1]["confirmed"]


def test_unsafe_state_does_not_publish_and_unknown_is_not_guessed():
    schedule = GaitSchedule(REFERENCE, INTERVALS)
    assert tick(schedule, 5., eligible=False)[0] is None
    assert tick(schedule, 5., eligible=False)[1]["reason"] == "waiting_for_eligible_state"
    unknown = GaitSchedule(REFERENCE, [{"start_m":0., "end_m":20., "gait":"unknown", "gait_code":None}])
    assert tick(unknown, 5.)[0] is None
    assert tick(unknown, 5.)[1]["target_code"] is None
    assert tick(GaitSchedule(REFERENCE, []), 0.)[0] is None


def test_reverse_route_and_start_inside_stairs():
    reverse = GaitSchedule(list(reversed(REFERENCE)), INTERVALS)
    assert reverse.tick({"x":16., "y":0.},0.,FLAT,0.,True)[0] == STAIRS
    inside = GaitSchedule(REFERENCE, [{"start_m":0., "end_m":3., "gait":"stairs", "gait_code":STAIRS},
                                     {"start_m":3., "end_m":20., "gait":"flat", "gait_code":FLAT}])
    assert tick(inside,0.)[0] == STAIRS
    assert tick(inside,4.,1.,STAIRS)[0] == FLAT


def test_nearby_stairs_merge_and_parallel_road_does_not_skip_progress():
    intervals = [
        {"start_m":0., "end_m":2., "gait":"flat", "gait_code":FLAT},
        {"start_m":2., "end_m":4., "gait":"stairs", "gait_code":STAIRS},
        {"start_m":4., "end_m":5., "gait":"flat", "gait_code":FLAT},
        {"start_m":5., "end_m":7., "gait":"stairs", "gait_code":STAIRS},
        {"start_m":7., "end_m":20., "gait":"flat", "gait_code":FLAT},
    ]
    schedule = GaitSchedule(REFERENCE, intervals)
    assert schedule.stairs == [[1.,8.]]
    assert tick(schedule,4.5,actual=STAIRS)[1]["target_code"] == STAIRS
    route = RouteStation([{"x":0.,"y":0.},{"x":20.,"y":0.},{"x":20.,"y":1.},{"x":0.,"y":1.}])
    assert route.update({"x":5.,"y":1.}) == 5.
    assert route.update({"x":4.,"y":0.}) == 5.


@pytest.mark.parametrize("change", [
    {"start_m":-1.}, {"end_m":21.}, {"start_m":True}, {"end_m":math.nan},
    {"gait_code":0x1003}, {"gait":"unknown"},
])
def test_bad_intervals_are_rejected(change):
    bad = [{**INTERVALS[0], **change}, *INTERVALS[1:]]
    with pytest.raises(ValueError):validate_intervals(REFERENCE,bad)


def task_node():
    status = {"ready":True, "localization_grade":"float", "coordinate_reference":{
        "frame_id":"map", "axes":"east_north", "origin_wgs84":ORIGIN,
        "map_offset_xy_m":[0.,0.], "antenna_xy_m":[0.,0.]}}
    node = SimpleNamespace(tasks=TaskStore(),state_lock=threading.RLock(),
        config={"fixed_arrival_m":3.,"float_arrival_m":3.},
        runtime={"gait_switching_enabled":True},status=lambda *_:status)
    return node,status


def request():
    return {"frame_id":"map", "reference_path":REFERENCE, "goal":REFERENCE[-1],
        "request_id":"c"*32, "map_id":"field-v1", "version":"a"*16,
        "origin_wgs84":ORIGIN, "intervals":INTERVALS}


def test_route_metadata_is_owned_and_retry_cannot_replace_it():
    node,status = task_node();p=request();task=submit(node,p)
    assert task["intervals"] == INTERVALS and task["map_version"] == "a"*16
    status["ready"]=False
    assert submit(node,p)["task_id"] == task["task_id"]
    with pytest.raises(ValueError):submit(node,{**p,"version":"b"*16})


def test_wrong_origin_and_unconfigured_gait_reject_before_task_replacement():
    node,status = task_node()
    with pytest.raises(ValueError):submit(node,{**request(),"origin_wgs84":{"latitude":0.,"longitude":0.}})
    node.runtime["gait_switching_enabled"]=False
    with pytest.raises(ValueError):submit(node,request())
    assert not node.tasks.tasks


def test_worker_gait_tick_does_not_pause_velocity_and_cancel_stops_retries(monkeypatch):
    from outdoor_nav.runtime import OutdoorExecutor
    import outdoor_nav.runtime as runtime
    node, status = task_node()
    task = submit(node, request())
    node.tasks.update(task["task_id"], state="navigating")
    node.owner = task["task_id"]
    node.gait_schedule = node.gait_task = None
    sent = []
    node.gait_pub = SimpleNamespace(publish=lambda msg: sent.append(msg.data))
    node.pose = lambda: {"x":4., "y":0., "age_s":.01}
    node.config["sensor_timeout_s"] = 1.
    node.stopping = False
    node.chassis = {"state":17,"gait":FLAT,"usage_mode":1,"motion_feedback_age_s":.01,
                    "usage_mode_age_s":.01,"gait_output_enabled":True}
    node.chassis_at = 100.
    node.status = lambda *_: {"health":{"chassis":True,"localization":True,"motion_enabled":True}}
    monkeypatch.setattr(runtime.time,"monotonic",lambda:100.1)
    OutdoorExecutor.gait_tick(node)
    assert sent == [STAIRS] and node.stopping is False
    assert not node.tasks.snapshot()["gait_status"]["confirmed"]
    node.tasks.cancel(task["task_id"])
    OutdoorExecutor.gait_tick(node)
    assert sent == [STAIRS] and node.gait_schedule is None
