"""Deployment-owned Nav2 configuration. No static map or raw GNSS dependency."""

import json
import math
import os
from pathlib import Path
import re

from .nav2_parameters import prepare, projection_config

DEFAULTS = {
    "lidar_enabled": True,
    "camera_enabled": False,
    "camera_frame_confirmed": False,
    "camera_expected_ground_z_m": -0.18,
    "camera_ground_tolerance_m": 0.05,
    "camera_ground_z_min_m": -0.45,
    "camera_ground_z_max_m": 0.15,
    "camera_static_min_hits": 4,
    "camera_static_dwell_s": 0.6,
    "camera_static_gap_s": 0.35,
    "camera_static_cell_m": 0.15,
    "camera_static_drift_m": 0.12,
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
    "gait_switch_timeout_s": 10.0,
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
    "camera_rgb": "/camera_gemini330/color/image_raw",
    "camera_depth": "/camera_gemini330/depth/image_raw",
    "camera_depth_info": "/camera_gemini330/depth/camera_info",
    "camera_color_info": "/camera_gemini330/color/camera_info",
    "camera_obstacles": "/outdoor/direct_nav/visual_obstacles",
    "camera_ground": "/outdoor/direct_nav/observed_ground",
    "camera_status": "/outdoor/direct_nav/depth_status",
    "camera_semantics": "/outdoor/vision/current_grid",
    "camera_costmap": "/outdoor/vision/filtered_costmap",
    "camera_scan": "/outdoor/camera_scan",
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
    allowed = {"params_file", "provider_ids", "topic_remap", "semantic_segmentation",
               "policy", "scan_projection", "velocity_output_topic", "gait_switching_enabled",
               "gait_output_topic", "global_grid", "run_dir", "http_port", "action_wait_s", "worker_python"}
    if set(config) - allowed:
        raise ValueError("Unknown service config fields: " + str(sorted(set(config) - allowed)))
    for field in ("policy", "topic_remap", "provider_ids"):
        if not isinstance(config.get(field, {}), dict):
            raise ValueError(field + " must be a mapping")
    providers = config.get("provider_ids", {})
    if set(providers) - {"cloud", "odom", "localization", "chassis"} or any(not isinstance(v, str) for v in providers.values()):
        raise ValueError("Unknown provider role or non-string provider id")
    if type(config.get("http_port", 19975)) is not int or not 1024 <= config.get("http_port", 19975) <= 65535:
        raise ValueError("http_port must be an integer in [1024, 65535]")
    wait = config.get("action_wait_s", 45.)
    if isinstance(wait, bool) or not isinstance(wait, (int, float)) or not math.isfinite(wait) or wait <= 0:
        raise ValueError("action_wait_s must be finite and positive")
    for field in ("run_dir", "worker_python"):
        if field in config and (not isinstance(config[field], str) or not config[field].strip()):
            raise ValueError(field + " must be a nonempty path")
    base = Path(base or os.environ.get("RBNX_INVOCATION_CWD", ".")).expanduser().resolve()
    if not isinstance(config.get("params_file"), str):
        raise ValueError("params_file must be a path string")
    path = Path(config.get("params_file", "")).expanduser()
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
        "gait_switch_timeout_s",
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
    if not (policy["lidar_enabled"] or policy["camera_enabled"]):
        raise ValueError("Enable at least one obstacle sensor")
    from .segmentation import validate_segmentation
    segmentation = validate_segmentation(config.get("semantic_segmentation", {}), base, policy["camera_enabled"])
    if not (-.6 <= policy["camera_expected_ground_z_m"] <= -.05
            and .005 <= policy["camera_ground_tolerance_m"] <= .1
            and policy["camera_ground_z_min_m"] < policy["camera_ground_z_max_m"]):
        raise ValueError("Invalid original camera ground calibration")
    if (type(policy["camera_static_min_hits"]) is not int or policy["camera_static_min_hits"] < 2
        or min(policy[k] for k in ("camera_static_dwell_s", "camera_static_gap_s", "camera_static_cell_m", "camera_static_drift_m")) <= 0):
        raise ValueError("Invalid camera stability policy")
    bindings = {**TOPICS, **config.get("topic_remap", {})}
    if set(bindings) - TOPICS.keys():
        raise ValueError("Unknown topic role")
    for value in bindings.values():
        topic(value)
    if bindings["camera_obstacles"] != "/outdoor/direct_nav/visual_obstacles":
        raise ValueError("Original depth node requires its guarded outdoor observation endpoint")
    if len({bindings[k] for k in ("cloud", "scan", "camera_scan", "camera_costmap", "camera_obstacles", "camera_ground", "camera_status")}) != 7:
        raise ValueError("Lidar and camera scan topics must be distinct")
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
    gait_enabled = config.get("gait_switching_enabled", False)
    if type(gait_enabled) is not bool:
        raise ValueError("gait_switching_enabled must be boolean")
    gait_output = topic(config.get("gait_output_topic", "/robonix/nomotion/outdoor_gait"))
    if gait_output in (*bindings.values(), output, "/GAIT"):
        raise ValueError("Gait requests need a separate guarded UInt32 output topic")
    normalized = {**config, "params_file": str(path.resolve())}
    nav2, derived, grid = prepare(normalized, bindings, policy)
    projection = projection_config(config, policy)
    policy.update(derived)
    return {
        **config,
        "params_file": str(path.resolve()),
        "policy": policy,
        "topic_remap": bindings,
        "velocity_output_topic": output,
        "gait_switching_enabled": gait_enabled,
        "gait_output_topic": gait_output,
        "global_grid": grid,  # Compatibility view derived from the deployment YAML.
        "nav2_params": nav2,
        "scan_projection": projection,
        "semantic_segmentation": segmentation,
        **({"run_dir": str((base / Path(config["run_dir"]).expanduser()).resolve())} if "run_dir" in config else {}),
    }


def load_runtime(path):
    return json.loads(Path(path).read_text())
