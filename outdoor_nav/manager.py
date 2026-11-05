"""Instance-owned lifecycle; starts no localization or hardware drivers."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from .configuration import validate


class RuntimeManager:
    def __init__(self):
        self.process = None
        self.lock = threading.RLock()
        self.runtime = None
        self.log = None

    def request(self, path, payload=None):
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError("Outdoor worker is not running")
        req = Request(
            f"http://127.0.0.1:{self.runtime.get('http_port', 19975)}" + path,
            data=None if payload is None else json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
        )
        try:
            with urlopen(req, timeout=3) as r:
                return json.load(r)
        except HTTPError as e:
            raise ValueError(json.load(e).get("error", str(e))) from e

    def start(self, config):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return
            runtime = validate(config)
            run = Path(
                runtime.get(
                    "run_dir",
                    Path(__file__).resolve().parents[1] / "rbnx-build/runtime",
                )
            ).resolve()
            run.mkdir(parents=True, exist_ok=True)
            # Refuse reuse while another worker already owns the HTTP endpoint.
            import socket

            with socket.socket() as probe:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                probe.bind(("127.0.0.1", runtime.get("http_port", 19975)))
            runtime["run_dir"] = str(run)
            self.runtime = runtime
            file = run / "runtime.json"
            file.write_text(json.dumps(runtime, indent=2))
            self.log = (run / "worker.log").open("a", buffering=1)
            env = {**os.environ, "OUTDOOR_SERVICE_INSTANCE": str(run)}
            self.process = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    str(Path(__file__).with_name("service.launch.py")),
                    "runtime:=" + str(file),
                ],
                stdin=subprocess.DEVNULL,
                stdout=self.log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=env,
            )
            # ros2 launch is a CLI entry point, not `python -m launch` on Humble.
            if self.process.poll() is not None:
                raise RuntimeError("Launch exited")
            (run / "owner.json").write_text(
                json.dumps({"pid": self.process.pid, "run_dir": str(run)})
            )
        try:
            deadline = time.monotonic() + runtime.get("action_wait_s", 45)
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    raise RuntimeError(
                        "Worker exited; inspect " + str(run / "worker.log")
                    )
                try:
                    health = self.request("/api/status")["health"]
                    if all(
                        health.get(k)
                        for k in ("actions", "planner_server", "controller_server")
                    ):
                        return
                except OSError:
                    pass
                time.sleep(0.1)
            raise TimeoutError("Outdoor Nav2 actions did not become ready")
        except Exception:
            self.shutdown()
            raise

    def active(self, value):
        self.request("/api/activation", {"active": value})
        if not value:
            deadline = time.monotonic() + 20
            stable = None
            while time.monotonic() < deadline:
                state = self.request("/api/status")
                task = state.get("task")
                v = state.get("measured_velocity")
                stopped = (
                    v
                    and state.get("velocity_fresh")
                    and v[0] <= self.runtime["policy"]["stopped_linear_mps"]
                    and v[1] <= self.runtime["policy"]["stopped_angular_rps"]
                )
                stable = (
                    (time.monotonic() if stable is None else stable)
                    if stopped
                    else None
                )
                if (
                    (not task or task["state"] in ("succeeded", "failed", "canceled"))
                    and stable
                    and time.monotonic() - stable
                    >= self.runtime["policy"]["stop_hold_s"]
                ):
                    return
                time.sleep(0.05)
            raise TimeoutError("Stop not confirmed from fresh measured velocity")

    def shutdown(self):
        with self.lock:
            p = self.process
            if p is None:
                return
            try:
                if p.poll() is None:
                    try:
                        self.request("/api/activation", {"active": False})
                    except Exception:
                        pass
                    os.killpg(p.pid, signal.SIGINT)
                    try:
                        p.wait(timeout=25)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid, signal.SIGTERM)
                        try:
                            p.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(p.pid, signal.SIGKILL)
                            p.wait(timeout=5)
            finally:
                # Cleanup only children with this instance's inherited marker.
                marker = (
                    "OUTDOOR_SERVICE_INSTANCE=" + self.runtime["run_dir"]
                ).encode()
                for proc in Path("/proc").iterdir():
                    if not proc.name.isdigit():
                        continue
                    try:
                        if (
                            marker in (proc / "environ").read_bytes().split(b"\0")
                            and (proc / "stat").read_text().split()[2] != "Z"
                        ):
                            os.kill(int(proc.name), signal.SIGTERM)
                    except (OSError, ProcessLookupError):
                        pass
                self.process = None
                if self.log:
                    self.log.close()
                    self.log = None
