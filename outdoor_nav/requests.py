"""Validated reference input and idempotent execution ownership, no road planner."""

import math
import re
from .http import ApiError
from .core.guidance import validate_point
from .core.gait import validate_intervals


def submit(node, payload):
    if not isinstance(payload, dict) or set(payload) - {
        "frame_id",
        "reference_path",
        "goal",
        "request_id",
        "arrival_tolerance_m",
        "localization_grade",
        "map_id", "version", "origin_wgs84", "intervals",
    }:
        raise ValueError("Unknown reference execution fields")
    if payload.get("frame_id") != "map":
        raise ValueError("Reference frame must be map")
    reference = payload.get("reference_path")
    if not isinstance(reference, list) or not 1 <= len(reference) <= 4096:
        raise ValueError("Reference needs 1..4096 finite points")
    reference = [validate_point(p) for p in reference]
    goal = validate_point(payload.get("goal"))
    if reference[-1] != goal:
        raise ValueError("Reference endpoint must equal the exact goal")
    if "yaw" in payload["goal"]:
        yaw = payload["goal"]["yaw"]
        if (
            isinstance(yaw, bool)
            or not isinstance(yaw, (int, float))
            or not math.isfinite(yaw)
        ):
            raise ValueError("Invalid goal yaw")
        goal["yaw"] = float(yaw)
    request_id = payload.get("request_id")
    if not isinstance(request_id, str) or not re.fullmatch("[a-f0-9]{32}", request_id):
        raise ValueError("request_id needs 32 lowercase hex characters")
    metadata = {"map_id": None, "map_version": None, "origin_wgs84": None, "intervals": []}
    if any(k in payload for k in ("map_id", "version", "origin_wgs84", "intervals")):
        map_id, version, origin = (payload.get(k) for k in ("map_id", "version", "origin_wgs84"))
        if not isinstance(map_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", map_id):
            raise ValueError("Invalid map_id")
        if not isinstance(version, str) or not re.fullmatch("[a-f0-9]{16,64}", version):
            raise ValueError("Invalid map content version")
        if (not isinstance(origin, dict) or set(origin) != {"latitude", "longitude"}
                or any(type(origin[k]) not in (float, int) or not math.isfinite(origin[k])
                       for k in origin)
                or not -90 <= origin["latitude"] <= 90 or not -180 <= origin["longitude"] <= 180):
            raise ValueError("Invalid map origin")
        metadata = {"map_id": map_id, "map_version": version, "origin_wgs84": dict(origin),
                    "intervals": validate_intervals(reference, payload.get("intervals", []))}
    grade = payload.get("localization_grade")
    if grade is not None and grade not in ("fixed", "float"):
        raise ValueError("Unknown localization grade")
    with node.state_lock:
        old = (
            node.tasks.snapshot(request_id) if request_id in node.tasks.tasks else None
        )
        # A retry reads its original execution even if feedback has since failed.
        # Never reinterpret omitted policy against the current localization grade.
        status = None if old else node.status(False)
        grade = grade or (
            old["localization_grade"] if old else status.get("localization_grade")
        )
        if grade not in ("fixed", "float"):
            raise ApiError(503, "No accepted localization grade")
        tolerance = payload.get(
            "arrival_tolerance_m",
            old["arrival_tolerance_m"] if old else node.config[grade + "_arrival_m"],
        )
        if (
            isinstance(tolerance, bool)
            or not isinstance(tolerance, (int, float))
            or not math.isfinite(tolerance)
            or tolerance <= 0
        ):
            raise ValueError("Invalid arrival tolerance")
        if old:
            if (
                old["goal"] != goal
                or old["reference_path"] != reference
                or old["arrival_tolerance_m"] != tolerance
                or old["localization_grade"] != grade
                or any(old.get(k) != v for k, v in metadata.items())
            ):
                raise ApiError(409, "request_id was used for a different execution")
            return old
        if status.get("localization_grade") not in ("fixed", "float"):
            raise ApiError(503, "New tasks require current RTK fixed or float; inertial continuation only")
        if metadata["map_id"] is not None:
            ref = status.get("coordinate_reference") or {}
            if (ref.get("frame_id") != "map" or ref.get("axes") != "east_north"
                    or ref.get("origin_wgs84") != metadata["origin_wgs84"]
                    or ref.get("map_offset_xy_m") != [0., 0.]
                    or ref.get("antenna_xy_m") != [0., 0.]):
                raise ApiError(409, "Route and live localization coordinate references differ")
            if ref.get("map_id") is not None and (ref.get("map_id") != metadata["map_id"]
                                                  or ref.get("version") != metadata["map_version"]):
                raise ApiError(409, "Route and localization map identities differ")
        if metadata["intervals"]:
            if any(i["gait"] == "unknown" for i in metadata["intervals"]):
                raise ApiError(409, "Unknown route gait; inspect the recording before execution")
            if not getattr(node, "runtime", {}).get("gait_switching_enabled", False):
                raise ApiError(503, "Automatic gait switching is not configured")
            if (node.config.get("require_chassis_ready", False)
                    and not getattr(node, "chassis", {}).get("gait_output_enabled", False)):
                raise ApiError(503, "Chassis gait output is not armed")
        if not status["ready"]:
            raise ApiError(503, "Navigation is not ready")
        with node.tasks.lock:
            node.tasks.submit(goal, task_id=request_id)
            node.tasks.update(
                request_id,
                reference_path=reference,
                arrival_tolerance_m=float(tolerance),
                localization_grade=grade,
                state="accepted",
                **metadata,
            )
            return node.tasks.snapshot(request_id)
