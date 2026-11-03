import ast
from pathlib import Path
import threading
from types import SimpleNamespace
import pytest
from outdoor_nav.configuration import validate
from outdoor_nav.core.tasks import TaskStore
from outdoor_nav.requests import submit

ROOT = Path(__file__).resolve().parents[1]


def cfg(**kw):
    return {"params_file": str(ROOT / "config/nav2_params.example.yml"), **kw}


def test_safe_configuration_and_no_manufacturer_odom():
    c = validate(cfg())
    assert c["velocity_output_topic"] == "/robonix/nomotion/outdoor_cmd_vel"
    assert not c["policy"]["motion_enabled"]
    for value in ("", "cmd_vel", "/bad//topic", "/bad/", None):
        with pytest.raises(ValueError):
            validate(cfg(velocity_output_topic=value))
    with pytest.raises(ValueError):
        validate(cfg(topic_remap={"odom": "/ODOM"}))
    with pytest.raises(ValueError):
        validate(cfg(policy={"max_speed_mps": float("nan")}))
    with pytest.raises(ValueError):
        validate(cfg(policy={"max_speed_mps": 0.5}))


def node():
    return SimpleNamespace(
        tasks=TaskStore(),
        state_lock=threading.RLock(),
        config={"fixed_arrival_m": 3.0, "float_arrival_m": 3.0},
        status=lambda *_: {"ready": True, "localization_grade": "fixed"},
    )


def payload():
    return {
        "frame_id": "map",
        "goal": {"x": 10.0, "y": 0.0},
        "reference_path": [
            {"x": 0.0, "y": 0.0},
            {"x": 0.0, "y": 10.0},
            {"x": 10.0, "y": 10.0},
            {"x": 10.0, "y": 0.0},
        ],
        "request_id": "a" * 32,
    }


def test_exact_reference_and_idempotence_and_replacement():
    n = node()
    p = payload()
    a = submit(n, p)
    assert (
        a["reference_path"] == p["reference_path"] and a["arrival_tolerance_m"] == 3.0
    )
    assert submit(n, p)["task_id"] == a["task_id"] and len(n.tasks.tasks) == 1
    with pytest.raises(ValueError):
        submit(n, {**p, "arrival_tolerance_m": 0.5})
    b = submit(n, {**p, "request_id": "b" * 32})
    assert (
        b["state"] == "accepted"
        and n.tasks.snapshot(a["task_id"])["state"] == "canceling"
    )


@pytest.mark.parametrize(
    "change",
    [
        {"frame_id": "odom"},
        {"goal": {"x": 9.0, "y": 0.0}},
        {"reference_path": []},
        {"reference_path": [{"x": float("inf"), "y": 0}]},
        {"request_id": "bad"},
        {"arrival_tolerance_m": 0},
        {"arrival_tolerance_m": True},
        {"localization_grade": "differential"},
    ],
)
def test_invalid_reference(change):
    with pytest.raises(ValueError):
        submit(node(), {**payload(), **change})


def test_unready_cannot_replace_running_execution():
    n = node()
    a = submit(n, payload())
    n.status = lambda *_: {"ready": False, "localization_grade": None}
    assert submit(n, payload())["task_id"] == a["task_id"]
    with pytest.raises(ValueError):
        submit(n, {**payload(), "request_id": "b" * 32})
    assert n.tasks.snapshot(a["task_id"])["state"] == "accepted"


def test_extracted_motion_methods_match_frozen_upstream():
    def methods(path, cls):
        tree = ast.parse(path.read_text())
        c = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
        return {
            n.name: n
            for n in c.body
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
        }

    old = methods(ROOT / "tests/upstream_executor.py.txt", "Navigation")
    new = methods(ROOT / "outdoor_nav/executor.py", "Executor")
    for name in new:
        if name == "costmap_snapshot":
            continue  # Imports point to the packaged core only.
        a, b = old[name], new[name]
        if name == "execute":
            # Exactly one seam: consume the submitted reference instead of generating it.
            assign = next(
                n
                for n in ast.walk(a)
                if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == "route" for t in n.targets)
            )
            snapshot = next(
                n
                for n in ast.walk(a)
                if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == "state" for t in n.targets)
            )
            snapshot.value.keywords[0].value = ast.Constant(value=True)
            assign.value = ast.parse(
                '{"reference_path": state["reference_path"], "goal": state["goal"]}',
                mode="eval",
            ).body
        if name == "guided_path":
            a.body[0].value.value = a.body[0].value.value.replace(
                "RPP path", "controller path"
            )
        assert ast.dump(a) == ast.dump(b), name
