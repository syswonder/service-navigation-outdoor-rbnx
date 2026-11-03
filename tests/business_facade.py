"""Test-only existing outdoorNav business/API facade, separate from service."""

import argparse
import json
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from outdoornav.api import serve, ApiError
from outdoornav.bundle import load_bundle
from outdoornav.outdoor import OutdoorService
from outdoornav.places import Places
from outdoornav.tasks import TaskStore
from outdoor_nav.client import ExecutionClient


def remote(path, payload=None):
    req = Request(
        "http://127.0.0.1:19975" + path,
        data=None if payload is None else json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
    )
    try:
        with urlopen(req, timeout=5) as r:
            return json.load(r)
    except HTTPError as e:
        raise ApiError(e.code, json.load(e).get("error", str(e))) from e


class Tasks:
    def __init__(self, client):
        self.client = client

    def snapshot(self, run_id=None):
        return self.client.status(run_id) if run_id else self.client.status()["task"]

    def cancel(self, run_id):
        return self.client.cancel(run_id)


class Facade:
    obstacles = None

    def __init__(self, bundle, config, places, endpoint="127.0.0.1:29975"):
        self.bundle = load_bundle(bundle)
        self.config = config
        self.places = Places(places, self.bundle)
        self.client = ExecutionClient(endpoint)
        self.tasks = Tasks(self.client)
        self.business = OutdoorService(
            self.bundle, self.places, TaskStore(), self.status, config
        )

    def status(self, *_):
        s = self.client.status()
        return {
            **s,
            "mode": "lynx",
            "run_id": "service-test",
            "planning_policy": self.business.planning_policy(),
            "scene_version": self.bundle["scene_version"],
            "route_version": self.bundle["route_version"],
            "localization": "ok" if s["health"]["localization"] else "waiting",
            "localization_mode": self.config["localization_mode"],
            "odometry_source": "rtk_doppler+wit_imu",
            "velocity_source": "ublox_doppler_and_wit_gyro",
            "heading": s.get("heading", {}),
            "feedback": s.get("feedback", {}),
            "not_ready_reasons": [
                {"code": x, "message": x} for x in s["not_ready_reasons"]
            ],
        }

    def costmap_snapshot(self, layer):
        return remote("/api/costmaps/" + layer)

    def map_snapshot(self):
        return self.bundle

    def preview(self, p):
        return self.business.preview(p)

    def submit(self, p):
        status = self.status(False)
        if not status["ready"]:
            raise ApiError(503, "Navigation is not ready")
        goal, _, route = self.business.validated(p)
        request = {
            "frame_id": "map",
            "goal": {k: v for k, v in goal.items() if k in ("x", "y", "yaw")},
            "reference_path": route["reference_path"],
            "request_id": p.get("request_id"),
            "localization_grade": status["localization_grade"],
            "arrival_tolerance_m": self.config[
                status["localization_grade"] + "_arrival_m"
            ],
        }
        try:
            run_id = self.client.follow_reference(request)
        except ValueError as error:
            raise ApiError(503, str(error)) from error
        return self.client.status(run_id)

    def set_active(self, v):
        remote("/api/activation", {"active": v})


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bundle")
    p.add_argument("--config")
    p.add_argument("--places")
    p.add_argument("--web")
    p.add_argument("--grpc-endpoint", default="127.0.0.1:29975")
    a = p.parse_args()
    config = json.load(open(a.config))
    facade = Facade(a.bundle, config, a.places, a.grpc_endpoint)
    http = serve(facade, port=19865, web_root=a.web)
    import signal, threading

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()
    http.shutdown()
    facade.client.close()


if __name__ == "__main__":
    main()
