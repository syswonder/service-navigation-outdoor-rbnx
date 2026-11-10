import pytest
from outdoor_nav.localization_policy import continuation_allowed, arrival_confirmed
from outdoor_nav.requests import submit
from test_contract import node, payload
from test_scene import route

pytest_plugins = ["test_scene"]


def test_only_existing_tasks_continue_and_no_inertial_arrival():
    status = {
        "ready": True,
        "grade": "inertial",
        "fusion": {"estimate_valid": True, "navigation_ready": True},
    }
    assert continuation_allowed(status, {"localization_grade": "fixed"})
    assert not continuation_allowed(status, None)
    assert not arrival_confirmed(status)
    status["ready"] = False
    assert not continuation_allowed(status, {"localization_grade": "fixed"})
    assert continuation_allowed(
        {"ready": True, "grade": "float"}, {"localization_grade": "fixed"}
    )
    assert not continuation_allowed(
        {"ready": True, "grade": "float"}, {"localization_grade": "fixed"}, False
    )


def test_request_cannot_spoof_grade_but_retry_remains_available():
    n = node()
    p = payload()
    accepted = submit(n, p)
    n.status = lambda *_: {"ready": True, "localization_grade": "inertial"}
    assert submit(n, p)["task_id"] == accepted["task_id"]
    for grade in (None, "fixed", "float"):
        request = {**p, "request_id": "b" * 32}
        if grade is not None:
            request["localization_grade"] = grade
        with pytest.raises(ValueError):
            submit(n, request)
    assert len(n.tasks.tasks) == 1


def test_scene_new_task_and_retry_during_inertial(nav):
    nav.submit_reference(route(), "e" * 32, {})
    nav.execution.value["localization_grade"] = "inertial"
    assert nav.submit_reference(route(), "e" * 32, {})["accepted"]
    with pytest.raises(ValueError):
        nav.submit_reference(route(), "f" * 32, {})


def test_scene_localization_record_is_saved_and_rate_limited(nav):
    import json

    nav.execution.value["fusion"] = {
        "stage": "inertial",
        "estimate_valid": True,
        "navigation_ready": True,
        "rtk_grade": "unavailable",
        "inertial": True,
        "inertial_elapsed_s": 4.0,
        "fusion_elapsed_s": 8.0,
        "recovery_id": 1,
    }
    nav.execution.value["localization_grade"] = "inertial"
    nav.status()
    nav.status()
    path = nav.run_dir / "localization.jsonl"
    lines = path.read_text().splitlines()
    assert len(lines) == 1
    row = json.loads(lines[0])
    assert (
        row["grade"] == "inertial"
        and row["inertial_elapsed_s"] == 4.0
        and row["fusion_elapsed_s"] == 8.0
    )
    assert row["rtk_grade"] == "unavailable"
    nav.execution.value["fusion"]["stage"] = "fixed"
    nav.execution.value["fusion"]["fusion_elapsed_s"] = 0.0
    nav.execution.value["localization_grade"] = "fixed"
    nav.status()
    assert len(path.read_text().splitlines()) == 2
    nav.localization_log.close()
