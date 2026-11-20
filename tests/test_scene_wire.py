"""Real generated gRPC and provider handlers; no ROS executor or hardware output."""

from concurrent import futures
from contextlib import contextmanager
import json
import threading
from types import SimpleNamespace

import grpc

from robonix_api.lifecycle import resolve_servicer, bind_user_handler
import robonix_contracts_pb2_grpc as contracts
from outdoor_nav import atlas_bridge as bridge
from outdoor_nav.client import ExecutionClient
from outdoor_nav.core.tasks import TaskStore
from outdoor_nav.requests import submit
from outdoor_nav.scene import AtlasExecution
from test_scene import nav, route


class Worker:
    def __init__(self, status):
        self.node = SimpleNamespace(tasks=TaskStore(), state_lock=threading.RLock(),
                                    config={"fixed_arrival_m":3., "float_arrival_m":3.},
                                    status=lambda *_: status)

    def request(self, path, payload=None):
        if path == "/api/status":
            return {**self.node.status(), "task": self.node.tasks.snapshot()}
        if path == "/api/tasks":
            return submit(self.node, payload)
        if path.endswith("/cancel"):
            return self.node.tasks.cancel(path.split("/")[-2])
        return self.node.tasks.snapshot(path.split("/")[-1])


def test_generated_provider_handoff_and_cancel(nav, monkeypatch):
    worker = Worker(nav.execution.value)
    monkeypatch.setattr(bridge, "manager", worker)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    bound = 0
    for contract, handler, _ in bridge.service._grpc_handlers:
        if not contract.startswith("robonix/service/navigation_outdoor/"):
            continue
        cls, method, add, _ = resolve_servicer(contract, contracts)
        add(bind_user_handler(cls, method, handler)(), server)
        bound += 1
    assert bound == 4
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()

    class Wire(AtlasExecution):
        @contextmanager
        def connection(self, action):
            client = ExecutionClient(f"127.0.0.1:{port}")
            try: yield client
            finally: client.close()

    try:
        nav.execution = Wire()
        worker.node.runtime = {"gait_switching_enabled":True}
        planned=route()
        request_id='d'*32
        accepted=nav.submit_reference(planned,request_id,{'goal':planned['goal']})
        task=nav.execution.status(accepted['run_id'])
        assert task['reference_path']==planned['reference_path']
        assert task['intervals']==planned['intervals']
        assert task['map_id']==planned['map_id'] and task['map_version']==planned['version']
        assert nav.submit_reference(planned,request_id,{'goal':planned['goal']})['run_id']==request_id
        assert nav.cancel(request_id)['state']=='canceling'
    finally:
        server.stop(0).wait()
