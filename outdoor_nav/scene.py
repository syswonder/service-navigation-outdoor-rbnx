"""Scene consumer of the independent navigation service; no device or velocity I/O."""

from contextlib import contextmanager
import json
import os
from pathlib import Path
import re
import tempfile
import threading
import time
from logging.handlers import RotatingFileHandler
import logging


from .http import ApiError

MAX_REQUEST_RECORDS = 1024
MAX_REQUEST_BYTES = 2 * 1024 * 1024


class AtlasExecution:
    def __init__(self, provider_id="outdoor_execution"):
        self.provider_id = provider_id

    @contextmanager
    def connection(self, action):
        from robonix_api import ATLAS
        from outdoor_nav.client import ExecutionClient

        contract = "robonix/service/navigation_outdoor/" + action
        found = ATLAS.find_unique_capability(
            contract_id=contract, provider_id=self.provider_id, transport="grpc"
        )
        channel = ATLAS.connect_capability(
            consumer_id="outdoor_scene", provider_id=found.provider_id,
            contract_id=contract, transport="grpc",
        )
        client = None
        try:
            client = ExecutionClient(channel.endpoint)
            yield client
        finally:
            if client is not None:
                client.close()
            channel.close()

    def status(self, run_id=""):
        with self.connection("status") as client:
            return client.status(run_id)

    def follow_reference(self, payload):
        with self.connection("follow_reference") as client:
            return client.follow_reference(payload)

    def cancel(self, run_id):
        with self.connection("cancel") as client:
            return client.cancel(run_id)


class AtlasLocalization:
    def __init__(self, config_path, provider_id="localization_outdoor"):
        self.config_path, self.provider_id = Path(config_path), provider_id

    def bind(self, snapshot):
        from robonix_api import ATLAS
        import grpc
        import lifecycle_pb2
        import robonix_contracts_pb2_grpc as contracts
        config = json.loads(self.config_path.read_text())
        expected = {"origin": snapshot["origin_wgs84"], "map_id": snapshot["map_id"],
                    "map_version": snapshot["version"], "require_map_reference": True}
        unchanged = all(config.get(k) == v for k, v in expected.items())
        config.update(expected)
        contract = "robonix/service/localization/driver"
        found = ATLAS.find_unique_capability(contract_id=contract, provider_id=self.provider_id, transport="grpc")
        ref = ATLAS.connect_capability(consumer_id="outdoor_scene", provider_id=found.provider_id,
                                      contract_id=contract, transport="grpc")
        try:
            with grpc.insecure_channel(ref.endpoint, options=[('grpc.enable_http_proxy', 0)]) as channel:
                driver = contracts.RobonixServiceLocalizationDriverStub(channel)
                for command in ((0, 1) if unchanged else (2, 0, 1)):
                    response = driver.Driver(lifecycle_pb2.Driver_Request(
                        command=command, config_json=json.dumps(config) if command == 0 else '{}'), timeout=25)
                    if not response.ok:
                        raise ValueError("Localization map binding failed: "+str(response))
        finally:
            ref.close()


class SceneNavigation:
    def __init__(self, run_dir="rbnx-build/scene_requests", execution=None, policy=None,
                 execution_enabled=False, localization_config_path=None):
        self.current_map = None
        self.localization_log = None
        self.localization_logged_at = -float("inf")
        self.localization_logged_stage = None
        self.localization = AtlasLocalization(localization_config_path) if localization_config_path else None
        self.execution = execution or AtlasExecution()
        self.execution_enabled = execution_enabled
        self.run_dir = Path(run_dir)
        self.lock = threading.RLock()
        self.policy = {"fixed_arrival_m": 3.0, "float_arrival_m": 3.0, **(policy or {})}

    def bind_map(self, snapshot):
        with self.lock:
            if self.execution_enabled:
                task = self.execution.status().get("task") or {}
                if task.get("state") in ("accepted", "planning", "navigating", "replanning", "canceling"):
                    raise ValueError("Confirm navigation has stopped before binding another map")
            if self.localization:
                self.localization.bind(snapshot)
            self.current_map = {k:snapshot[k] for k in ('map_id', 'version', 'origin_wgs84')}

    def status(self, *_):
        value = self.execution.status()
        reference = value.get("coordinate_reference") or {}
        current = self.current_map
        matches = (
            current is not None and reference.get("frame_id") == "map"
            and reference.get("axes") == "east_north"
            and reference.get("origin_wgs84") == current["origin_wgs84"]
            and reference.get("map_id") == current["map_id"]
            and reference.get("version") == current["version"]
            and reference.get("map_offset_xy_m") == [0.0, 0.0]
            and reference.get("antenna_xy_m") == [0.0, 0.0]
            and reference.get("reference_point") == "base_link"
        )
        value = {**value, "coordinate_reference_matches_map": matches}
        if not self.execution_enabled:
            value["ready"] = False
            value["not_ready_reasons"] = [*value.get("not_ready_reasons", []), "scene_execution_disabled"]
        if not matches:
            value["ready"] = False
            value["not_ready_reasons"] = [*value.get("not_ready_reasons", []),
                                         "map_coordinate_reference_mismatch"]
        fusion = value.get("fusion")
        if isinstance(fusion, dict):
            self.record_localization(value)
        return value

    def record_localization(self, value):
        fusion = value["fusion"]
        stage = (value.get("localization_grade"), fusion.get("rtk_grade"), fusion.get("stage"), fusion.get("recovery_id"))
        with self.lock:
            now = time.monotonic()
            if now-self.localization_logged_at < 1. and stage == self.localization_logged_stage:
                return
            if self.localization_log is None:
                self.run_dir.mkdir(parents=True, exist_ok=True)
                self.localization_log = RotatingFileHandler(self.run_dir/"localization.jsonl", maxBytes=2*1024*1024, backupCount=2, encoding="utf-8")
            fields = ("stage", "rtk_grade", "estimate_valid", "navigation_ready", "inertial", "inertial_elapsed_s",
                      "fusion_elapsed_s", "fusion_timer_reference", "outage_s", "recovery_id", "uncertainty")
            record = {"recorded_at_unix_s": time.time(), "map_id": (value.get("coordinate_reference") or {}).get("map_id"),
                      "grade": value.get("localization_grade"), **{k: fusion.get(k) for k in fields}}
            self.localization_log.emit(logging.LogRecord("outdoor.localization", logging.INFO, "", 0,
                                                        json.dumps(record, allow_nan=False), (), None))
            self.localization_logged_at, self.localization_logged_stage = now, stage

    def _follow_recorded(self, request_id, source, build):
        if not self.execution_enabled:
            raise ApiError(403, "Scene execution is disabled in this deployment profile")
        if not isinstance(request_id, str) or not re.fullmatch("[a-f0-9]{32}", request_id):
            raise ValueError("request_id needs 32 lowercase hex characters")
        with self.lock:
            record_path = self.run_dir / (request_id + ".json")
            if record_path.exists():
                if record_path.stat().st_size > MAX_REQUEST_BYTES:
                    raise ApiError(409, "Saved request exceeds the retention size limit")
                record = json.loads(record_path.read_text())
                if record["source_payload"] != source:
                    raise ApiError(409, "request_id was used for a different goal or policy")
                request = record["execution_payload"]
            else:
                if sum(1 for _ in self.run_dir.glob("*.json")) >= MAX_REQUEST_RECORDS:
                    raise ApiError(503, "Navigation request storage is full; archive completed records")
                request = build()
                encoded = json.dumps({"source_payload":source,"execution_payload":request}, allow_nan=False).encode()
                if len(encoded) > MAX_REQUEST_BYTES:
                    raise ApiError(413, "Navigation request record exceeds 2 MiB")
                self.run_dir.mkdir(parents=True, exist_ok=True)
                with tempfile.NamedTemporaryFile(mode="wb", dir=self.run_dir, delete=False) as f:
                    f.write(encoded)
                    temporary = f.name
                os.replace(temporary, record_path)
            run_id = self.execution.follow_reference(request)
            return {"accepted":True,"run_id":run_id,**request}

    def submit_reference(self, route, request_id, source):
        def build():
            status = self.status()
            if not status["ready"]:
                raise ApiError(503, "Navigation is not ready: " + str(status.get("not_ready_reasons", [])))
            if (route.get("frame_id") != "map"
                    or route.get("origin_wgs84") != (self.current_map or {}).get("origin_wgs84")
                    or (self.current_map and any(route.get(k) != self.current_map[k] for k in ('map_id','version')))):
                raise ApiError(409, "Route differs from the loaded map and localization reference")
            grade = status.get("localization_grade")
            if grade not in ("fixed","float"):
                raise ApiError(503, "No accepted localization grade")
            return {"frame_id":"map","reference_path":route["reference_path"],"goal":route["goal"],
                    "request_id":request_id,"localization_grade":grade,
                    "arrival_tolerance_m":self.policy[grade+"_arrival_m"],
                    "map_id":route["map_id"],"version":route["version"],
                    "origin_wgs84":route["origin_wgs84"],"intervals":route.get("intervals",[])}
        return self._follow_recorded(request_id,source,build)

    def cancel(self, run_id):
        if not isinstance(run_id, str) or not re.fullmatch("[a-f0-9]{32}", run_id):
            raise ValueError("Invalid run_id")
        return self.execution.cancel(run_id)
