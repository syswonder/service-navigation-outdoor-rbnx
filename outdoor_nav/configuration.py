"""Deployment-owned Nav2 configuration. No static map or raw GNSS dependency."""

import json
import math
import os
from pathlib import Path
import re

from .nav2_parameters import prepare, projection_config
from .ground_reference import load_ground_reference

DEFAULTS = {
    "lidar_enabled": True,
    "ranger_descent_enabled": False,
    "ranger_step_limit_m": 0.075,
    "ranger_grade_limit_deg": 15.0,
    "ranger_brake_mps2": 0.55,
    "ranger_depth_timeout_s": 0.35,
    "camera_enabled": False,
    "camera_geometry_enabled": True,
    "camera_geometry_backend": "legacy",
    "camera_blind_zone_m": 0.0,
    "camera_max_range_m": 4.0,
    "camera_frame_confirmed": False,
    # Mount pose of the depth camera in base_link: the driver publishes its own
    # link -> optical frames, so only this one static transform is missing.
    "camera_frame": "camera_gemini330_link",
    "camera_mount_xyz_m": [0.0, 0.0, 0.0],
    "camera_mount_rpy_deg": [0.0, 0.0, 0.0],
    "camera_expected_ground_z_m": None,
    "camera_ground_tolerance_m": 0.05,
    "camera_ground_z_min_m": -0.45,
    "camera_ground_z_max_m": 0.15,
    "camera_static_min_hits": 4,
    "camera_static_dwell_s": 0.6,
    "camera_static_gap_s": 0.35,
    "camera_static_cell_m": 0.15,
    "camera_static_drift_m": 0.12,
    "sensor_timeout_s": 1.0,
    "pose_timeout_s": 3.0,
    "localization_timeout_s": 3.0,
    "command_timeout_s": 0.5,
    "navigation_recovery_timeout_s": 15.0,
    "fixed_arrival_m": 3.0,
    "float_arrival_m": 3.0,
    "max_speed_mps": 0.4,
    "max_angular_speed_rps": 0.4,
    "footprint_radius_m": 0.6,
    "cloud_min_height_m": -0.3,
    "cloud_max_height_m": 0.7,
    "cloud_min_height_near_m": 0.2,
    "cloud_min_height_far_m": 0.5,
    "cloud_height_ramp_m": 8.0,
    "lidar_height_m": None,
    "stairs_disable_obstacle_layer": True,
    "scan_range_min_m": 0.5,
    "planner_patience_s": 60.0,
    "controller_patience_s": 60.0,
    "stopped_linear_mps": 0.10,
    "stopped_angular_rps": 0.06,
    "stop_hold_s": 1.0,
    "gait_switch_timeout_s": 5.0,
    "gait_prepare_timeout_s": 15.0,
    "transform_tolerance_s": 3.0,
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
    "cycle_costmap": "/outdoor/cycle/costmap",
    "cycle_ack": "/outdoor/cycle/costmap_ack",
    "imu": "/wit_10axis/data",
    "visual_terrain": "/outdoor/vision/terrain",
    "lidar_terrain": "/outdoor/lidar/terrain",
    "terrain_state": "/outdoor/terrain_state",
    "filtered_cloud": "/outdoor/cloud_for_scan",
    "camera_rgb": "/camera_gemini330/color/image_raw",
    "camera_depth": "/camera_gemini330/depth/image_raw",
    "camera_depth_info": "/camera_gemini330/depth/camera_info",
    "camera_color_info": "/camera_gemini330/color/camera_info",
    "camera_obstacles": "/outdoor/direct_nav/visual_obstacles",
    "camera_ground": "/outdoor/direct_nav/observed_ground",
    "camera_status": "/outdoor/direct_nav/depth_status",
    "camera_semantics": "/outdoor/vision/current_grid",
    "camera_road_mask": "/outdoor/vision/road_mask",
    "camera_labels": "/outdoor/vision/semantic_labels",
    "camera_confidence": "/outdoor/vision/semantic_confidence",
    "camera_segmentation_status": "/semantic/segmentation_status",
    "camera_overlay": "/semantic/debug_overlay",
    "camera_costmap": "/outdoor/vision/filtered_costmap",
    "camera_scan": "/outdoor/camera_scan",
    "cloud": "/airy_201/points",
    "scan": "/outdoor/scan",
    "obstacle_scan": "/outdoor/obstacle_scan",
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
    allowed = {"ground_height_file", "decision_cycle", "terrain", "params_file", "provider_ids", "topic_remap", "semantic_segmentation",
               "policy", "scan_projection", "velocity_output_topic", "gait_switching_enabled",
               "gait_output_topic", "global_grid", "run_dir", "http_port", "action_wait_s", "worker_python"}
    if set(config) - allowed:
        raise ValueError("Unknown service config fields: " + str(sorted(set(config) - allowed)))
    for field in ("policy", "topic_remap", "provider_ids", "terrain"):
        if not isinstance(config.get(field, {}), dict):
            raise ValueError(field + " must be a mapping")
    providers = config.get("provider_ids", {})
    if set(providers) - {"cloud", "odom", "localization", "chassis", "camera", "imu"} or any(not isinstance(v, str) for v in providers.values()):
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
    ground_file, ground = load_ground_reference(config, base)
    config = {**config, "ground_height_file": ground_file,
              "policy": {**config.get("policy", {}), "camera_expected_ground_z_m": ground,
                         "lidar_height_m": -ground},
              "terrain": {**config.get("terrain", {}), "ground_z_m": ground}}
    if not isinstance(config.get("params_file"), str):
        raise ValueError("params_file must be a path string")
    path = Path(config.get("params_file", "")).expanduser()
    if not config.get("params_file"):
        raise ValueError("params_file is required")
    path = path if path.is_absolute() else base / path
    if not path.is_file():
        raise ValueError("params_file does not exist")
    from .cycle_config import validate_cycle
    cycle = validate_cycle(config.get("decision_cycle", {}))
    policy = {**DEFAULTS, **config.get("policy", {})}
    # One localization age budget feeds pose, status and velocity consumers.
    # Accept the old alias only when it does not create two conflicting limits.
    configured = config.get("policy", {})
    if "pose_timeout_s" in configured:
        if ("localization_timeout_s" in configured
                and configured["pose_timeout_s"] != configured["localization_timeout_s"]):
            raise ValueError("Use one localization_timeout_s for pose and localization")
        policy["localization_timeout_s"] = configured["pose_timeout_s"]
    policy["pose_timeout_s"] = policy["localization_timeout_s"]
    policy["gait_prepare_timeout_s"] = configured.get(
        "gait_prepare_timeout_s", policy["navigation_recovery_timeout_s"])
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
        "pose_timeout_s",
        "localization_timeout_s",
        "command_timeout_s",
        "navigation_recovery_timeout_s",
        "fixed_arrival_m",
        "float_arrival_m",
        "stop_hold_s",
        "gait_switch_timeout_s",
        "gait_prepare_timeout_s",
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
        and 0 <= policy["cloud_min_height_near_m"] <= policy["cloud_min_height_far_m"]
        and policy["cloud_height_ramp_m"] > 0
        and policy["cloud_max_height_m"] > policy["cloud_min_height_near_m"]
        and 0 < policy["lidar_height_m"] <= 2
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
    if policy["camera_geometry_backend"] not in ("legacy", "rtabmap"):
        raise ValueError("camera_geometry_backend must be legacy or rtabmap")
    if policy["camera_enabled"] and not (policy["camera_geometry_enabled"] or segmentation["enabled"]):
        raise ValueError("Camera mode requires geometry detection or semantic segmentation")
    if not (isinstance(policy["camera_frame"], str) and policy["camera_frame"].strip()
            and all(isinstance(policy[key], list) and len(policy[key]) == 3
                    and not any(isinstance(v, bool) or not isinstance(v, (int, float))
                                or not math.isfinite(v) for v in policy[key])
                    for key in ("camera_mount_xyz_m", "camera_mount_rpy_deg"))):
        raise ValueError("camera_frame and camera_mount_xyz_m/camera_mount_rpy_deg must be set")
    if not 0 <= policy["camera_blind_zone_m"] <= policy["camera_max_range_m"]:
        raise ValueError("camera_blind_zone_m must be nonnegative and within camera_max_range_m")
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
    if len({bindings[k] for k in ('camera_rgb', 'camera_road_mask', 'camera_labels', 'camera_confidence', 'camera_overlay')}) != 5:
        raise ValueError('Camera RGB and segmentation outputs must use distinct topics')
    if bindings["camera_obstacles"] != "/outdoor/direct_nav/visual_obstacles":
        raise ValueError("Original depth node requires its guarded outdoor observation endpoint")
    if len({bindings[k] for k in ("cloud", "scan", "obstacle_scan", "camera_scan", "camera_costmap", "camera_obstacles", "camera_ground", "camera_status")}) != 8:
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
    from .terrain_config import validate_terrain
    if not 0.5 <= policy["camera_max_range_m"] <= 10:
        raise ValueError("camera_max_range_m must be within [0.5, 10]")
    normalized = {**config, "params_file": str(path.resolve())}
    nav2, derived, grid = prepare(normalized, bindings, policy)
    if cycle["enabled"]:
        internal = {bindings['cycle_costmap'],bindings['cycle_ack']}
        inputs = {v for k,v in bindings.items() if k not in ('cycle_costmap','cycle_ack')}
        if len(internal)!=2 or internal & inputs:
            raise ValueError('Cycle snapshot and acknowledgment topics must be distinct from sensor inputs')
        configure_cycle_nav2(nav2, bindings, cycle)
        grid = dict(origin=[-cycle['grid_half_width_m']]*2,
                    width=int(math.ceil(2*cycle['grid_half_width_m'])),
                    height=int(math.ceil(2*cycle['grid_half_width_m'])),
                    resolution=cycle['grid_resolution_m'],rolling=True)
    projection = projection_config(config, policy)
    policy.update(derived)
    terrain = validate_terrain(config.get("terrain", {}), policy, segmentation, gait_enabled)
    if cycle["enabled"]:
        terrain["observation_timeout_s"] = cycle["period_s"]
    if policy["ranger_descent_enabled"]:
        if (not policy["lidar_enabled"] or not policy["camera_frame_confirmed"]
                or not policy["footprint_confirmed"] or not policy["cloud_frame_confirmed"]):
            raise ValueError("Ranger descent requires verified camera, lidar and footprint")
        if cycle["enabled"] or terrain["enabled"] or gait_enabled:
            raise ValueError("Ranger descent uses the continuous flat controller")
        if not (0 < policy["ranger_step_limit_m"] <= .15
                and 0 < policy["ranger_grade_limit_deg"] <= 30
                and 0 < policy["ranger_brake_mps2"] <= 1
                and 0 < policy["ranger_depth_timeout_s"] <= policy["sensor_timeout_s"]):
            raise ValueError("Invalid Ranger descent limits")
    if terrain["enabled"] and terrain["can_climb_stairs"]:
        from .nav2_parameters import parameters
        stairs = parameters(nav2, "controller_server").get("StairsFollowPath", {})
        if (stairs.get("plugin") != "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
                or stairs.get("use_collision_detection") is not True):
            raise ValueError("Terrain stairs require StairsFollowPath RPP with use_collision_detection: true")
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
        "terrain": terrain,
        "decision_cycle": cycle,
        **({"run_dir": str((base / Path(config["run_dir"]).expanduser()).resolve())} if "run_dir" in config else {}),
    }


def load_runtime(path):
    return json.loads(Path(path).read_text())

def configure_cycle_nav2(nav2, topics, cycle):
    """Explicit opt-in replaces live sensor layers with one frozen odom snapshot."""
    from .nav2_parameters import parameters
    local = parameters(nav2, 'local_costmap')
    if 'camera_layer' in local['plugins']:
        local['camera_layer']['max_age_s'] = cycle['period_s']
    block = parameters(nav2, 'global_costmap')
    extras=set(block['plugins'])-{'lidar_layer','camera_layer','inflation_layer'}
    if extras:
        raise ValueError('Decision cycle cannot freeze additional costmap layers: '+str(sorted(extras)))
    block['global_frame']='odom'
    block['rolling_window']=True
    block['width']=block['height']=int(math.ceil(2*cycle['grid_half_width_m']))
    block['resolution']=cycle['grid_resolution_m']
    block['origin_x']=block['origin_y']=-cycle['grid_half_width_m']
    block['plugins']=['cycle_layer','inflation_layer']
    block['cycle_layer']={'plugin':'outdoor_camera_layer::CameraLayer','enabled':True,
        'grid_topic':topics['cycle_costmap'],'ack_topic':topics['cycle_ack'],
        'max_age_s':cycle['period_s']}
