"""Read deploy-owned Nav2 parameters without synthesizing a robot profile."""
import math
import warnings
from pathlib import Path

import yaml


def number(value, field, minimum=None, maximum=None):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or (minimum is not None and value < minimum)
            or (maximum is not None and value > maximum)):
        raise ValueError(f"Invalid {field}: {value!r}")
    return value


def parameters(document, node):
    try:
        block = document[node]
        if node.endswith("costmap"):
            block = block[node]
        result = block["ros__parameters"]
        if not isinstance(result, dict):
            raise TypeError()
        return result
    except (KeyError, TypeError):
        raise ValueError(f"params_file requires complete {node}.ros__parameters") from None


def projection_config(config, policy):
    raw = config.get("scan_projection", {})
    defaults = {"enabled": policy["lidar_enabled"], "target_frame": "base_link",
                "min_height_m": -.3, "max_height_m": .3, "range_min_m": .5,
                "range_max_m": 8., "transform_tolerance_s": .2}
    if not isinstance(raw, dict) or set(raw) - defaults.keys():
        raise ValueError("Unknown scan_projection fields or non-mapping value")
    result = {**defaults, **raw}
    legacy = config.get("policy", {})
    for old, new in (("cloud_min_height_m", "min_height_m"), ("cloud_max_height_m", "max_height_m"),
                     ("scan_range_min_m", "range_min_m"), ("transform_tolerance_s", "transform_tolerance_s")):
        if old in legacy:
            if new in raw:
                raise ValueError(f"Use scan_projection.{new} without legacy policy.{old}")
            warnings.warn(f"policy.{old} is deprecated; use scan_projection.{new}", FutureWarning)
            result[new] = legacy[old]
    if type(result["enabled"]) is not bool or result["enabled"] != policy["lidar_enabled"]:
        raise ValueError("scan_projection.enabled must match policy.lidar_enabled for the 3D lidar input")
    if result["target_frame"] != "base_link":
        raise ValueError("Outdoor execution currently requires scan_projection.target_frame=base_link")
    for key in ("min_height_m", "max_height_m", "range_min_m", "range_max_m", "transform_tolerance_s"):
        number(result[key], "scan_projection." + key)
    if not (result["min_height_m"] < result["max_height_m"]
            and 0 <= result["range_min_m"] < result["range_max_m"]
            and result["transform_tolerance_s"] >= 0):
        raise ValueError("Invalid scan_projection bounds")
    return result


def migrate_legacy(document, config):
    """Explicit old overrides remain available with a migration warning."""
    policy = config.get("policy", {})
    controller = parameters(document, "controller_server")
    follow = controller["FollowPath"]
    smoother = parameters(document, "velocity_smoother")
    maps = [parameters(document, name) for name in ("global_costmap", "local_costmap")]
    moved = {"max_speed_mps", "max_angular_speed_rps", "footprint_radius_m",
             "planner_tolerance_m", "transform_tolerance_s", "inflation_padding_m",
             "inflation_cost_scaling", "local_track_unknown", "obstacle_layer_enabled"} & policy.keys()
    if moved:
        warnings.warn("Deprecated Nav2 policy fields: " + ", ".join(sorted(moved))
                      + "; move them into params_file", FutureWarning)
    if "max_speed_mps" in moved:
        speed = number(policy["max_speed_mps"], "policy.max_speed_mps", .000001, .4)
        follow.update(max_vel_x=speed, max_speed_xy=speed)
        smoother["max_velocity"][0] = speed
        if "StairsFollowPath" in controller:
            stairs = controller["StairsFollowPath"]
            stairs["desired_linear_vel"] = min(stairs["desired_linear_vel"], speed)
    if "max_angular_speed_rps" in moved:
        speed = number(policy["max_angular_speed_rps"], "policy.max_angular_speed_rps", .000001, .4)
        follow["max_vel_theta"] = speed
        smoother["max_velocity"][2], smoother["min_velocity"][2] = speed, -speed
        if "StairsFollowPath" in controller:
            stairs = controller["StairsFollowPath"]
            stairs["rotate_to_heading_angular_vel"] = min(stairs["rotate_to_heading_angular_vel"], speed)
    if "planner_tolerance_m" in moved:
        parameters(document, "planner_server")["GridBased"]["tolerance"] = policy["planner_tolerance_m"]
    for costmap in maps:
        if "footprint_radius_m" in moved:
            if costmap.get("footprint"):
                raise ValueError("Remove legacy footprint_radius_m when params_file uses a polygon")
            costmap["robot_radius"] = policy["footprint_radius_m"]
        if "transform_tolerance_s" in moved:
            costmap["transform_tolerance"] = policy["transform_tolerance_s"]
            follow["transform_tolerance"] = policy["transform_tolerance_s"]
        if "inflation_padding_m" in moved:
            costmap["inflation_layer"]["inflation_radius"] = costmap["robot_radius"] + policy["inflation_padding_m"]
        if "inflation_cost_scaling" in moved:
            costmap["inflation_layer"]["cost_scaling_factor"] = policy["inflation_cost_scaling"]
        if "obstacle_layer_enabled" in moved:
            for layer in ("lidar_layer", "camera_layer"):
                if layer in costmap:
                    costmap[layer]["enabled"] = policy["obstacle_layer_enabled"]
    if "local_track_unknown" in moved:
        maps[1]["track_unknown_space"] = policy["local_track_unknown"]
    if "global_grid" in config:
        warnings.warn("global_grid is deprecated; configure global_costmap in params_file", FutureWarning)
        grid = config["global_grid"]
        if (not isinstance(grid, dict) or set(grid) - {"origin", "width", "height", "resolution", "rolling"}
                or not {"origin", "width", "height", "resolution"} <= grid.keys()
                or not isinstance(grid["origin"], list) or len(grid["origin"]) != 2
                or type(grid.get("rolling", False)) is not bool):
            raise ValueError("Invalid legacy global_grid")
        for value in [*grid["origin"], grid["width"], grid["height"], grid["resolution"]]:
            number(value, "global_grid")
        maps[0].update(origin_x=grid["origin"][0], origin_y=grid["origin"][1],
                       width=math.ceil(grid["width"]), height=math.ceil(grid["height"]), resolution=grid["resolution"],
                       rolling_window=grid.get("rolling", False))


def prepare(config, topics, policy):
    document = yaml.safe_load(Path(config["params_file"]).read_text())
    if not isinstance(document, dict):
        raise ValueError("params_file must be a Nav2 YAML mapping")
    for node in ("planner_server", "controller_server", "velocity_smoother", "global_costmap", "local_costmap"):
        parameters(document, node)
    try:
        migrate_legacy(document, config)
    except KeyError as exc:
        raise ValueError(f"Incomplete Nav2 configuration: missing {exc}") from None
    replacements = {"__ROBONIX_ODOM_TOPIC__": topics["odom"],
                    "__ROBONIX_SCAN_TOPIC__": topics["scan"],
                    "__ROBONIX_SCAN_CLOUD_TOPIC__": topics["cloud"],
                    "__ROBONIX_CAMERA_COSTMAP_TOPIC__": topics["camera_costmap"]}

    def bind(value):
        if isinstance(value, dict):
            return {key: bind(item) for key, item in value.items()}
        if isinstance(value, list):
            return [bind(item) for item in value]
        if isinstance(value, str):
            for token, topic in replacements.items():
                value = value.replace(token, topic)
            if "__ROBONIX_" in value:
                raise ValueError(f"Unresolved Nav2 topic token: {value}")
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError("params_file contains a nonfinite numeric value")
        return value

    document = bind(document)
    controller = parameters(document, "controller_server")
    smoother = parameters(document, "velocity_smoother")
    try:
        plugins = controller["controller_plugins"]
        if not isinstance(plugins, list) or "FollowPath" not in plugins:
            raise ValueError("Outdoor executor requires the FollowPath controller id")
        for plugin in plugins:
            if not isinstance(controller[plugin].get("plugin"), str):
                raise ValueError(f"Missing controller plugin: {plugin}")
        follow = controller["FollowPath"]
        if follow["plugin"] != "dwb_core::DWBLocalPlanner":
            raise ValueError("Outdoor executor currently supports DWB FollowPath parameters")
        linear = min(number(follow["max_vel_x"], "FollowPath.max_vel_x", .000001, .4),
                     number(follow["max_speed_xy"], "FollowPath.max_speed_xy", .000001, .4))
        angular = number(follow["max_vel_theta"], "FollowPath.max_vel_theta", .000001, .4)
        for field in ("max_velocity", "min_velocity", "max_accel", "max_decel"):
            vector = smoother[field]
            if not isinstance(vector, list) or len(vector) != 3:
                raise ValueError(f"velocity_smoother.{field} requires three finite numbers")
            for value in vector:
                number(value, "velocity_smoother." + field)
        linear = min(linear, number(smoother["max_velocity"][0], "smoother linear limit", .000001))
        angular = min(angular, number(smoother["max_velocity"][2], "smoother yaw limit", .000001),
                      number(-smoother["min_velocity"][2], "smoother negative yaw limit", .000001))
        if config.get("gait_switching_enabled", False) and "StairsFollowPath" not in plugins:
            raise ValueError("Gait switching requires deploy-owned StairsFollowPath parameters")
        # Derived guard limits never rewrite the selected controller/smoother.
        derived = {"max_speed_mps": linear, "max_angular_speed_rps": angular}
        for name, frame in (("global_costmap", "map"), ("local_costmap", "odom")):
            costmap = parameters(document, name)
            if costmap.get("global_frame") != frame or costmap.get("robot_base_frame") != "base_link":
                raise ValueError(f"{name} must use {frame}/base_link for the outdoor executor")
            for field in ("width", "height", "resolution"):
                number(costmap[field], name + "." + field, .000001)
            if any(type(costmap[field]) is not int for field in ("width", "height")):
                raise ValueError("Costmap width and height must be positive integer metres")
            for field in ("rolling_window", "track_unknown_space"):
                if type(costmap.get(field)) is not bool:
                    raise ValueError(f"{name}.{field} must be boolean")
            for field in ("origin_x", "origin_y"):
                number(costmap.get(field, 0.), name + "." + field)
            if costmap["width"] * costmap["height"] / costmap["resolution"]**2 > 16000000:
                raise ValueError("Costmap exceeds 16 million cells")
            footprint = costmap.get("footprint")
            if footprint:
                points = yaml.safe_load(footprint) if isinstance(footprint, str) else footprint
                if (not isinstance(points, list) or len(points) < 3
                        or any(not isinstance(p, list) or len(p) != 2 for p in points)):
                    raise ValueError("Invalid costmap footprint polygon")
                radius = max(math.hypot(*(number(v, "footprint") for v in p)) for p in points)
            else:
                radius = number(costmap["robot_radius"], name + ".robot_radius", .1, 2)
            number(radius, name + " footprint radius", .1, 2)
            layers = costmap["plugins"]
            if not isinstance(layers, list) or len(layers) != len(set(layers)):
                raise ValueError("Costmap plugins must be a list without duplicates")
            for sensor in ("lidar", "camera"):
                layer = sensor + "_layer"
                if policy[sensor + "_enabled"]:
                    if layer not in layers:
                        raise ValueError(f"Enabled {sensor} requires {name}.{layer}")
                    block = costmap[layer]
                    expected = "nav2_costmap_2d::ObstacleLayer" if sensor == "lidar" else "outdoor_camera_layer::CameraLayer"
                    if block.get("plugin") != expected or block.get("enabled", True) is not True:
                        raise ValueError(f"Invalid or disabled {name}.{layer}")
                    if sensor == "lidar" and block.get("combination_method", 1) != 1:
                        raise ValueError("Lidar layer must merge independent costs using maximum")
                    if sensor == "camera":
                        if block.get("grid_topic") != topics["camera_costmap"]:
                            raise ValueError("camera_layer.grid_topic must bind the filtered camera costmap")
                        number(block.get("max_age_s"), "camera_layer.max_age_s", .000001)
                    else:
                        sources = block.get("observation_sources", "").split()
                        if not sources or any(block.get(s, {}).get("topic") != topics["scan"] for s in sources):
                            raise ValueError("lidar_layer sources must bind the independent lidar scan")
                elif layer in layers:
                    layers.remove(layer)
            # Other layers and all their tuning remain deployment-owned.
            if "inflation_layer" not in layers:
                raise ValueError(f"{name} requires a deploy-owned inflation_layer")
            for field in ("inflation_radius", "cost_scaling_factor"):
                number(costmap["inflation_layer"][field], "inflation_layer." + field, .000001)
        global_map = parameters(document, "global_costmap")
        grid = {"origin": [global_map.get("origin_x", 0.), global_map.get("origin_y", 0.)],
                "width": global_map["width"], "height": global_map["height"],
                "resolution": global_map["resolution"], "rolling": global_map.get("rolling_window", False)}
        return document, derived, grid
    except (KeyError, TypeError) as exc:
        raise ValueError(f"Incomplete or invalid params_file: {exc}") from None
