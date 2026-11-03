import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace
from outdoor_nav.manager import RuntimeManager


def test_deactivate_waits_for_task_and_measured_stop():
    state = {"active": True, "calls": 0}
    m = RuntimeManager()
    m.runtime = {
        "policy": {
            "stopped_linear_mps": 0.03,
            "stopped_angular_rps": 0.03,
            "stop_hold_s": 0.01,
        }
    }

    def request(path, payload=None):
        if payload is not None:
            state["active"] = payload["active"]
            return {"active": state["active"]}
        state["calls"] += 1
        moving = state["calls"] < 3
        return {
            "task": {"state": "canceling" if moving else "canceled"},
            "measured_velocity": [0.1 if moving else 0.0, 0.0],
            "velocity_fresh": True,
        }

    m.request = request
    m.active(False)
    assert not state["active"] and state["calls"] >= 3


def test_only_guard_receives_final_output_remap():
    # Global /cmd_vel remapping would bypass the controller's raw topic and
    # create two publishers on the protected output. Keep it child-specific.
    from pathlib import Path

    import ast

    tree = ast.parse(
        (Path(__file__).parents[1] / "outdoor_nav/service.launch.py").read_text()
    )
    assert any(
        isinstance(node, ast.comprehension)
        and any(
            isinstance(test, ast.Compare)
            and isinstance(test.ops[0], ast.NotEq)
            and any(
                isinstance(value, ast.Constant) and value.value == "/cmd_vel"
                for value in test.comparators
            )
            for test in node.ifs
        )
        for node in ast.walk(tree)
    )
    pairs = {
        tuple(item.value for item in node.elts)
        for node in ast.walk(tree)
        if isinstance(node, ast.Tuple)
        and all(isinstance(item, ast.Constant) for item in node.elts)
    }
    assert ("cmd_vel", "/outdoor/controller_command") in pairs
    assert ("cmd_vel_smoothed", "/navigation/controller_command") in pairs
