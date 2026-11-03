"""Real SDK, generated bindings and real local gRPC transport; worker is a test double."""

import asyncio
from concurrent import futures
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest

ROOT = Path(__file__).parents[1]
sys.path[:0] = [
    str(ROOT / "rbnx-build/codegen/proto_gen"),
    str(ROOT / "rbnx-build/codegen/robonix_mcp_types"),
]
pytest.importorskip("robonix_api")
pytest.importorskip("outdoor_execution_pb2")
import grpc
import lifecycle_pb2
import outdoor_execution_pb2 as pb
import robonix_contracts_pb2_grpc as rpc
from robonix_api.lifecycle import (
    build_lifecycle_servicer,
    resolve_servicer,
    bind_user_handler,
)
from outdoor_nav import atlas_bridge as bridge


class Worker:
    def __init__(self):
        self.active_value = False
        self.started = False
        self.current = None

    def start(self, config):
        self.started = True

    def shutdown(self):
        self.started = False

    def active(self, value):
        self.active_value = value

    def request(self, path, payload=None):
        if path == "/api/tasks":
            assert self.active_value
            self.current = {"task_id": payload["request_id"], "state": "accepted"}
            return self.current
        if path == "/api/status":
            return {"ready": self.active_value, "task": self.current}
        if path.endswith("/cancel"):
            self.current["state"] = "canceled"
            return self.current
        return self.current


def test_real_sdk_driver_and_rpc_mcp_wire(monkeypatch):
    worker = Worker()
    monkeypatch.setattr(bridge, "manager", worker)
    monkeypatch.setattr(bridge.service, "declare_ros2_topic", lambda *a, **k: None)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    lifecycle = build_lifecycle_servicer(
        "robonix/service/navigation_outdoor",
        rpc,
        lifecycle_pb2.Driver_Response,
        on_init=bridge.init,
        on_activate=bridge.activate,
        on_deactivate=bridge.deactivate,
        on_shutdown=bridge.shutdown,
    )
    assert lifecycle is not None
    lifecycle[1](lifecycle[0], server)
    for contract, handler, _ in bridge.service._grpc_handlers:
        info = resolve_servicer(contract, rpc)
        assert info is not None
        cls, method, add, _ = info
        add(bind_user_handler(cls, method, handler)(), server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    channel = grpc.insecure_channel(f"127.0.0.1:{port}")
    try:
        driver = rpc.RobonixServiceNavigationOutdoorDriverStub(channel)
        config = {
            "params_file": str(ROOT / "config/nav2_params.example.yml"),
            "topic_remap": {
                "cloud": "/test/cloud",
                "odom": "/test/rtk_imu_odom",
                "localization": "/test/localization",
            },
        }
        result = driver.Driver(
            lifecycle_pb2.Driver_Request(command=0, config_json=json.dumps(config)),
            timeout=5,
        )
        assert (
            result.ok
            and result.state == "inactive"
            and worker.started
            and not worker.active_value
        )
        assert driver.Driver(lifecycle_pb2.Driver_Request(command=1), timeout=5).ok
        payload = {"request_id": "a" * 32}
        follow = rpc.RobonixServiceNavigationOutdoorFollowReferenceStub(channel)
        accepted = follow.FollowReference(
            pb.FollowReference_Request(request_json=json.dumps(payload)), timeout=5
        )
        assert accepted.accepted and accepted.run_id == "a" * 32
        status = rpc.RobonixServiceNavigationOutdoorStatusStub(channel).Status(
            pb.Status_Request(run_id="a" * 32), timeout=5
        )
        assert status.known and status.state == "accepted"
        response = asyncio.run(
            bridge.service._mcp_app.call_tool("status", {"run_id": "a" * 32})
        )
        assert "accepted" in str(response)
        cancel = rpc.RobonixServiceNavigationOutdoorCancelStub(channel).Cancel(
            pb.Cancel_Request(run_id="a" * 32), timeout=5
        )
        assert cancel.accepted and worker.current["state"] == "canceled"
        # Invalid config fails before changing worker state.
        bad = driver.Driver(
            lifecycle_pb2.Driver_Request(
                command=0, config_json='{"velocity_output_topic":"bad"}'
            ),
            timeout=5,
        )
        assert not bad.ok and bad.state == "error"
        monkeypatch.setattr(bridge.ATLAS, "find_capability", lambda **kw: [])
        deferred = driver.Driver(
            lifecycle_pb2.Driver_Request(
                command=0,
                config_json=json.dumps({"params_file": config["params_file"]}),
            ),
            timeout=5,
        )
        assert not deferred.ok and deferred.state == "deferred"
        assert driver.Driver(lifecycle_pb2.Driver_Request(command=2), timeout=5).ok
        assert not worker.active_value
        assert driver.Driver(lifecycle_pb2.Driver_Request(command=3), timeout=5).ok
        assert not worker.started
    finally:
        channel.close()
        server.stop(0).wait()
