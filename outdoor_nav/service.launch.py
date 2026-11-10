"""Pure deployment launch: no localizer, drivers, static map or business server."""

import json
import math
import os
from pathlib import Path
import sys
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def nodes(context):
    runtime_file = LaunchConfiguration("runtime").perform(context)
    runtime = json.loads(Path(runtime_file).read_text())
    robot = runtime["policy"]
    worker_python = runtime.get("worker_python", sys.executable)
    params = yaml.safe_load(Path(runtime["params_file"]).read_text())
    params["planner_server"]["ros__parameters"]["GridBased"]["tolerance"] = robot[
        "planner_tolerance_m"
    ]
    params["controller_server"]["ros__parameters"]["FollowPath"][
        "transform_tolerance"
    ] = robot["transform_tolerance_s"]
    params["controller_server"]["ros__parameters"]["FollowPath"][
        "trans_stopped_velocity"
    ] = robot["stopped_linear_mps"]
    controllers = params["controller_server"]["ros__parameters"]
    controllers["StairsFollowPath"] = {
        "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
        "desired_linear_vel": min(.15, robot["max_speed_mps"]),
        "lookahead_dist": .5, "min_lookahead_dist": .25, "max_lookahead_dist": .6,
        "use_velocity_scaled_lookahead_dist": False,
        "rotate_to_heading_angular_vel": min(.2, robot["max_angular_speed_rps"]),
        "use_rotate_to_heading": True, "allow_reversing": False,
        "max_angular_accel": .6, "transform_tolerance": robot["transform_tolerance_s"],
        "use_collision_detection": False, "use_cost_regulated_linear_velocity_scaling": False,
        "use_regulated_linear_velocity_scaling": True,
        "min_approach_linear_velocity": .05,
    }
    controllers["controller_plugins"] = ["FollowPath", "StairsFollowPath"]
    grid = {
        **runtime["global_grid"],
        "width": runtime["global_grid"]["width"] / runtime["global_grid"]["resolution"],
        "height": runtime["global_grid"]["height"]
        / runtime["global_grid"]["resolution"],
    }
    for name, rolling, frame in (
        ("global_costmap", False, "map"),
        ("local_costmap", True, "odom"),
    ):
        costmap = {
            "use_sim_time": False,
            "global_frame": frame,
            "robot_base_frame": "base_link",
            "rolling_window": rolling or runtime["global_grid"].get("rolling", False),
            # Unknown cells ahead of the robot must not be treated as collisions;
            # the planner keeps its own allow_unknown policy on the global grid.
            "track_unknown_space": robot["local_track_unknown"] if rolling else True,
            "update_frequency": 5.0 if rolling else 2.0,
            "publish_frequency": 5.0 if rolling else 2.0,
            "always_send_full_costmap": True,
            "transform_tolerance": robot["transform_tolerance_s"],
            "resolution": float(grid["resolution"]),
            "robot_radius": robot["footprint_radius_m"],
            "width": 8 if rolling else math.ceil(grid["width"] * grid["resolution"]),
            "height": 8 if rolling else math.ceil(grid["height"] * grid["resolution"]),
            "plugins": ["obstacle_layer", "inflation_layer"],
            "obstacle_layer": {
                "plugin": "nav2_costmap_2d::ObstacleLayer",
                "enabled": robot["obstacle_layer_enabled"],
                "observation_sources": "scan"
                if robot["obstacle_layer_enabled"]
                else "",
                "scan": {
                    "topic": "/outdoor/scan",
                    "data_type": "LaserScan",
                    "marking": True,
                    "clearing": True,
                    "inf_is_valid": True,
                    "obstacle_max_range": 6.0,
                    "raytrace_max_range": 8.0,
                },
            },
            "inflation_layer": {
                "plugin": "nav2_costmap_2d::InflationLayer",
                "inflation_radius": robot["footprint_radius_m"]
                + robot["inflation_padding_m"],
                "cost_scaling_factor": robot["inflation_cost_scaling"],
            },
        }
        if not rolling:
            costmap.update(
                origin_x=float(grid["origin"][0]), origin_y=float(grid["origin"][1])
            )
        params[name] = {name: {"ros__parameters": costmap}}
    params["controller_server"]["ros__parameters"]["FollowPath"].update(
        max_vel_x=robot["max_speed_mps"], max_speed_xy=robot["max_speed_mps"]
    )
    params["velocity_smoother"]["ros__parameters"]["max_velocity"][0] = robot[
        "max_speed_mps"
    ]
    params["controller_server"]["ros__parameters"]["FollowPath"]["max_vel_theta"] = (
        robot["max_angular_speed_rps"]
    )
    params["velocity_smoother"]["ros__parameters"]["max_velocity"][2] = robot[
        "max_angular_speed_rps"
    ]
    params["velocity_smoother"]["ros__parameters"]["min_velocity"][2] = -robot[
        "max_angular_speed_rps"
    ]
    output = Path(runtime["run_dir"])
    output.mkdir(parents=True, exist_ok=True)
    effective = output / "effective_nav2.yaml"
    params["velocity_smoother"]["ros__parameters"]["velocity_timeout"] = robot[
        "command_timeout_s"
    ]
    effective.write_text(yaml.safe_dump(params))
    policy = runtime["topic_remap"]
    remaps = [
        ("/tf", policy["tf"]),
        ("/tf_static", policy["tf_static"]),
        ("/outdoor/odom", policy["odom"]),
        ("/outdoor/localization", policy["localization"]),
        ("/outdoor/chassis_status", policy["chassis"]),
        ("/outdoor/scan", policy["scan"]),
        ("/cmd_vel", runtime["velocity_output_topic"]),
    ]
    rosargs = ["--ros-args"] + [arg for a, b in remaps for arg in ("-r", a + ":=" + b)]
    children = [
        ExecuteProcess(
            cmd=[
                worker_python,
                "-m",
                "outdoor_nav.runtime",
                "--runtime",
                runtime_file,
                *rosargs,
            ],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                worker_python,
                "-m",
                "outdoor_nav.guard",
                *rosargs,
                "-r",
                "/outdoor/scan:=/outdoor/raw_scan",
                "-p",
                "robot_config:=" + runtime_file,
            ],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                worker_python,
                "-m",
                "outdoor_nav.cloud_height_filter",
                "--ros-args",
                "-r",
                "cloud_in:=" + policy["cloud"],
                "-p",
                "near_m:=" + repr(robot["cloud_min_height_near_m"]),
                "-p",
                "far_m:=" + repr(robot["cloud_min_height_far_m"]),
                "-p",
                "ramp_m:=" + repr(robot["cloud_height_ramp_m"]),
                "-p",
                "max_m:=" + repr(robot["cloud_max_height_m"]),
                "-p",
                "lidar_height_m:=" + repr(robot["lidar_height_m"]),
            ],
            output="screen",
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            remappings=[("cloud_in", "/outdoor/cloud_for_scan"),
                         ("scan", "/outdoor/raw_scan")],
            parameters=[
                {
                    "target_frame": "base_link",
                    "transform_tolerance": robot["transform_tolerance_s"],
                    # Height gating happens in cloud_height_filter; keep the
                    # projector's own band out of the way.
                    "min_height": -1.0e6,
                    "max_height": 1.0e6,
                    "range_min": robot["scan_range_min_m"]
                    or robot["footprint_radius_m"],
                    "range_max": 8.0,
                    "use_inf": True,
                }
            ],
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[str(effective)],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=[str(effective)],
            remappings=[("cmd_vel", "/outdoor/controller_command")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            parameters=[str(effective)],
            remappings=[
                ("cmd_vel", "/outdoor/controller_command"),
                ("cmd_vel_smoothed", "/navigation/controller_command"),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="outdoor_lifecycle",
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": [
                        "planner_server",
                        "controller_server",
                        "velocity_smoother",
                    ],
                }
            ],
        ),
    ]
    return [
        GroupAction(
            actions=[
                *[SetRemap(src=a, dst=b) for a, b in remaps if a != "/cmd_vel"],
                RegisterEventHandler(
                    OnProcessExit(
                        on_exit=[
                            EmitEvent(
                                event=Shutdown(reason="Outdoor service child exited")
                            )
                        ]
                    )
                ),
                *children,
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument("runtime"), OpaqueFunction(function=nodes)]
    )
