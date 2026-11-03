"""Deployment-owned Nav2 configuration. No static map or raw GNSS dependency."""

import json
import math
import os
from pathlib import Path
import re

DEFAULTS = {
    "sensor_timeout_s": 1.0,
    "command_timeout_s": 0.5,
    "navigation_recovery_timeout_s": 15.0,
    "fixed_arrival_m": 3.0,
    "float_arrival_m": 3.0,
    "max_speed_mps": 0.4,
    "max_angular_speed_rps": 0.4,
    "footprint_radius_m": 0.6,
    "cloud_min_height_m": -0.3,
    "cloud_max_height_m": 0.3,
    "scan_range_min_m": 0.5,
    "planner_patience_s": 60.0,
    "controller_patience_s": 60.0,
    "stopped_linear_mps": 0.03,
    "stopped_angular_rps": 0.03,
    "stop_hold_s": 1.0,
    "transform_tolerance_s": 0.2,
    "inflation_padding_m": 0.1,
    "inflation_cost_scaling": 5.0,
    "planner_tolerance_m": 0.2,
    "local_track_unknown": True,
    "obstacle_layer_enabled": True,
    "allow_float_during_fixed_task": True,
    "motion_enabled": False,
    "require_chassis_ready": True,
    "calibrated": False,
    "antenna_offset_confirmed": False,
    "footprint_confirmed": False,
    "cloud_frame_confirmed": False,
    "feedback_confirmed": False,
    "odometry_confirmed": False,
    "localization_mode": "rtk_imu",
    "heading_source": "wit_imu",
}
TOPICS = {
    "cloud": "/airy_201/points",
    "scan": "/outdoor/scan",
    "odom": "/outdoor/odom",
    "localization": "/outdoor/localization",
    "chassis": "/outdoor/chassis_status",
    "tf": "/outdoor/tf",
    "tf_static": "/tf_static",
}


def topic(value):
    if not isinstance(value, str) or not re.fullmatch(
        r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*", value
    ):
        raise ValueError("Expected an absolute ROS topic name")
    return value


def validate(config, base=None):
    if not isinstance(config, dict):
        raise ValueError("Service config must be an object")
    base = Path(base or os.environ.get("RBNX_INVOCATION_CWD", ".")).resolve()
    path = Path(config.get("params_file", ""))
    if not config.get("params_file"):
        raise ValueError("params_file is required")
    path = path if path.is_absolute() else base / path
    if not path.is_file():
        raise ValueError("params_file does not exist")
    policy = {**DEFAULTS, **config.get("policy", {})}
    if set(policy) - DEFAULTS.keys():
        raise ValueError(
            "Unknown policy keys: " + str(sorted(set(policy) - DEFAULTS.keys()))
        )
    for key, default in DEFAULTS.items():
        value = policy[key]
        if type(default) is bool:
            if type(value) is not bool:
                raise ValueError(key + " must be boolean")
        elif isinstance(default, (int, float)):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(key + " must be finite")
    for key in (
        "sensor_timeout_s",
        "command_timeout_s",
        "navigation_recovery_timeout_s",
        "fixed_arrival_m",
        "float_arrival_m",
        "stop_hold_s",
        "footprint_radius_m",
        "stopped_linear_mps",
        "stopped_angular_rps",
        "transform_tolerance_s",
    ):
        if policy[key] <= 0:
            raise ValueError(key + " must be positive")
    if not (
        0 < policy["max_speed_mps"] <= 0.4
        and 0 < policy["max_angular_speed_rps"] <= 0.4
        and policy["cloud_min_height_m"] < policy["cloud_max_height_m"]
        and 0 <= policy["scan_range_min_m"] < 8
        and 0 <= policy["planner_patience_s"] <= 300
        and 0 < policy["controller_patience_s"] <= 300
        and 0.1 <= policy["footprint_radius_m"] <= 2
        and policy["fixed_arrival_m"] <= policy["float_arrival_m"]
    ):
        raise ValueError("Invalid outdoor navigation policy")
    if policy["localization_mode"] not in ("rtk_imu", "external_odom") or policy[
        "heading_source"
    ] not in ("wit_imu", "configured"):
        raise ValueError("Unsupported feedback source")
    bindings = {**TOPICS, **config.get("topic_remap", {})}
    if set(bindings) - TOPICS.keys():
        raise ValueError("Unknown topic role")
    for value in bindings.values():
        topic(value)
    if bindings["odom"] == "/ODOM":
        raise ValueError("Unreliable manufacturer /ODOM is not a supported input")
    output = topic(
        config.get("velocity_output_topic", "/robonix/nomotion/outdoor_cmd_vel")
    )
    if output in bindings.values() or output in (
        "/outdoor/safe_command",
        "/navigation/controller_command",
        "/outdoor/controller_command",
    ):
        raise ValueError("Velocity input/output must not form a loop")
    grid = config.get(
        "global_grid",
        {
            "origin": [-100.0, -100.0],
            "width": 200.0,
            "height": 200.0,
            "resolution": 0.1,
        },
    )
    if (
        not isinstance(grid, dict)
        or set(grid) != {"origin", "width", "height", "resolution"}
        or not isinstance(grid["origin"], list)
        or len(grid["origin"]) != 2
    ):
        raise ValueError("Invalid global_grid")
    for value in [*grid["origin"], grid["width"], grid["height"], grid["resolution"]]:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ValueError("Invalid grid value")
    if (
        min(grid["width"], grid["height"], grid["resolution"]) <= 0
        or grid["width"] * grid["height"] / grid["resolution"] ** 2 > 16000000
    ):
        raise ValueError("Invalid grid dimensions")
    return {
        **config,
        "params_file": str(path.resolve()),
        "policy": policy,
        "topic_remap": bindings,
        "velocity_output_topic": output,
        "global_grid": grid,
    }


def load_runtime(path):
    return json.loads(Path(path).read_text())
