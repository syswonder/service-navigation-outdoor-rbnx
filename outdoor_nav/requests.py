"""Validated reference input and idempotent execution ownership, no road planner."""

import math
import re
from .http import ApiError
from .core.guidance import validate_point


def submit(node, payload):
    if not isinstance(payload, dict) or set(payload) - {
        "frame_id",
        "reference_path",
        "goal",
        "request_id",
        "arrival_tolerance_m",
        "localization_grade",
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
            ):
                raise ApiError(409, "request_id was used for a different execution")
            return old
        if not status["ready"]:
            raise ApiError(503, "Navigation is not ready")
        with node.tasks.lock:
            node.tasks.submit(goal, task_id=request_id)
            node.tasks.update(
                request_id,
                reference_path=reference,
                arrival_tolerance_m=float(tolerance),
                localization_grade=grade,
            )
            return node.tasks.snapshot(request_id)
