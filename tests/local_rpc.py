"""Local-only demo transport for the SAME provider handlers; no Atlas required.

The isolated fixture already owns the worker lifecycle. Production uses
outdoor_nav.atlas_bridge through Robonix Driver/Atlas instead of this launcher.
"""

import argparse
from concurrent import futures
import json
from pathlib import Path
import signal
import threading
from urllib.request import Request, urlopen
from urllib.error import HTTPError

import grpc
from robonix_api.lifecycle import resolve_servicer, bind_user_handler
import robonix_contracts_pb2_grpc as contracts
from outdoor_nav import atlas_bridge as bridge
from outdoor_nav.manager import RuntimeManager


class WorkerConnection:
    def __init__(self, runtime):
        self.runtime = runtime

    def request(self, path, payload=None):
        url = f"http://127.0.0.1:{self.runtime['http_port']}" + path
        request = Request(
            url,
            data=None if payload is None else json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
        )
        try:
            with urlopen(request, timeout=5) as response:
                result = json.load(response)
        except HTTPError as error:
            raise ValueError(json.load(error).get("error", str(error))) from error
        if path == "/api/tasks":
            with (Path(self.runtime["run_dir"]).parent / "route-handoff.jsonl").open(
                "a"
            ) as log:
                log.write(
                    json.dumps(
                        {
                            "transport": "grpc",
                            "run_id": result["task_id"],
                            "submitted": payload,
                            "accepted_reference": result["reference_path"],
                        }
                    )
                    + "\n"
                )
        return result

    active = RuntimeManager.active


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", required=True)
    parser.add_argument("--port", type=int, default=29975)
    args = parser.parse_args()
    bridge.manager = WorkerConnection(json.loads(Path(args.runtime).read_text()))
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=6))
    for contract, handler, _ in bridge.service._grpc_handlers:
        info = resolve_servicer(contract, contracts)
        if info is None:
            raise RuntimeError("Missing generated contract: " + contract)
        cls, method, add, _ = info
        add(bind_user_handler(cls, method, handler)(), server)
    bound = server.add_insecure_port(f"127.0.0.1:{args.port}")
    if bound != args.port:
        raise RuntimeError("Could not bind local demo gRPC endpoint")
    server.start()
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    print(f"LOCAL_GRPC_READY 127.0.0.1:{args.port}", flush=True)
    try:
        stopping.wait()
    finally:
        server.stop(3).wait()


if __name__ == "__main__":
    main()
