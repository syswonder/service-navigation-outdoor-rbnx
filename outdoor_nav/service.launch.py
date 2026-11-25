"""Pure deployment launch: no localizer, drivers, static map or business server."""

import json
from copy import deepcopy
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
    from outdoor_nav.vision_launch import vision_nodes
    worker_python = runtime.get("worker_python", sys.executable)
    # Validated deploy-owned YAML: topics are bound and disabled layers pruned.
    # Never reconstruct costmaps, controllers or overwrite the robot's tuning.
    params = runtime["nav2_params"]
    ranger = robot.get("ranger_descent_enabled", False)
    if ranger and robot["ranger_descent_mode"] == "staged":
        from outdoor_nav.core.ground_support import ACK_TOPIC, GRID_TOPIC
        from outdoor_nav.core.ground_support import SEGMENT_SPEED
        params["planner_server"]["ros__parameters"]["GridBased"].update(allow_unknown=False, tolerance=0.)
        controller = params["controller_server"]["ros__parameters"]["FollowPath"]
        controller["max_vel_x"] = min(controller["max_vel_x"], SEGMENT_SPEED)
        controller["max_speed_xy"] = min(controller["max_speed_xy"], SEGMENT_SPEED)
        controller["xy_goal_tolerance"] = min(controller.get("xy_goal_tolerance", .15), .15)
        smoother = params["velocity_smoother"]["ros__parameters"]
        smoother["max_velocity"][0] = min(smoother["max_velocity"][0], SEGMENT_SPEED)
        for name in ("global_costmap", "local_costmap"):
            costmap = params[name][name]["ros__parameters"]
            costmap["track_unknown_space"] = True
            costmap["update_frequency"] = max(10., costmap["update_frequency"])
            costmap["plugins"].insert(costmap["plugins"].index("inflation_layer"), "ranger_support")
            costmap["ranger_support"] = {
                "plugin": "outdoor_camera_layer::CameraLayer", "grid_topic": GRID_TOPIC,
                "ack_topic": ACK_TOPIC, "max_age_s": robot["ranger_depth_timeout_s"],
            }
        # Limit the plugin's full-window merge to a near-field rolling map.
        params["global_costmap"]["global_costmap"]["ros__parameters"].update(
            rolling_window=True, width=16, height=16)
    elif ranger:
        from outdoor_nav.core.ground_support import SEGMENT_SPEED
        controller = params["controller_server"]["ros__parameters"]
        descent = deepcopy(controller["FollowPath"])
        for key in ("max_vel_x", "max_speed_xy"):
            descent[key] = min(descent[key], SEGMENT_SPEED)
        for key in ("xy_goal_tolerance", "RotateToGoal.xy_goal_tolerance"):
            descent[key] = min(descent.get(key, .15), .15)
        controller["controller_plugins"].append("RangerFollowPath")
        controller["RangerFollowPath"] = descent
        checker = deepcopy(controller["goal_checker"])
        checker["xy_goal_tolerance"] = min(checker.get("xy_goal_tolerance", .25), .17)
        controller["goal_checker_plugins"].append("ranger_goal_checker")
        controller["ranger_goal_checker"] = checker
    output = Path(runtime["run_dir"])
    output.mkdir(parents=True, exist_ok=True)
    effective = output / "effective_nav2.yaml"
    effective.write_text(yaml.safe_dump(params))
    policy = runtime["topic_remap"]
    remaps = [
        ("/tf", policy["tf"]),
        ("/tf_static", policy["tf_static"]),
        ("/outdoor/odom", policy["odom"]),
        ("/outdoor/localization", policy["localization"]),
        ("/outdoor/chassis_status", policy["chassis"]),
        ("/outdoor/scan", policy["scan"]),
        ("/outdoor/obstacle_scan", policy["obstacle_scan"]),
        ("/outdoor/camera_scan", policy["camera_scan"]),
        ("/cmd_vel", runtime["velocity_output_topic"]),
    ]
    rosargs = ["--ros-args"] + [arg for a, b in remaps for arg in ("-r", a + ":=" + b)]
    projection = runtime["scan_projection"]
    terrain = runtime.get("terrain", {})
    filter_cloud = robot["lidar_enabled"]
    camera_children, camera_optional = vision_nodes(runtime, worker_python, rosargs) if robot["camera_enabled"] else ([], [])
    ranger_children = []
    if ranger:
        from outdoor_nav.core.ground_support import DEPTH_TOPIC
        depth_parameters = {
            "allow_outdoor_navigation": True, "alignment_verified": robot["camera_frame_confirmed"],
            "geometry_primary": True, "registered_only": True, "registered_points_topic": DEPTH_TOPIC,
            "max_age": robot["ranger_depth_timeout_s"], "max_range": robot["camera_max_range_m"],
        }
        depth_file = output / "ranger_depth.yaml"
        depth_file.write_text(yaml.safe_dump({"/**": {"ros__parameters": depth_parameters}}))
        depth_remaps = [
            ("/camera_gemini330/depth/image_raw", policy["camera_depth"]),
            ("/camera_gemini330/depth/camera_info", policy["camera_depth_info"]),
            ("/camera_gemini330/color/camera_info", policy["camera_color_info"]),
            ("/direct_nav/visual_obstacles", "/outdoor/direct_nav/visual_obstacles"),
        ]
        args = [value for source, target in depth_remaps for value in ("-r", source + ":=" + target)]
        ranger_children = [
            ExecuteProcess(cmd=[worker_python, "-m", "outdoor_nav.vision_nodes", "depth",
                                 *rosargs, *args, "--params-file", str(depth_file),
                                 "-r", "__node:=ranger_registered_depth"], output="screen"),
            ExecuteProcess(cmd=[worker_python, "-m", "outdoor_nav.ground_support_node",
                                 runtime_file, *rosargs], output="screen"),
        ]
    children = [
        *ranger_children,
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
                "-p",
                "robot_config:=" + runtime_file,
            ],
            output="screen",
        ),
        *([ExecuteProcess(cmd=[worker_python, "-m", "outdoor_nav.cloud_height_filter", runtime_file, *rosargs], output="screen")]
          if robot["lidar_enabled"] and filter_cloud else []),
        *([ExecuteProcess(cmd=[worker_python, "-m", "outdoor_nav.terrain_perception", runtime_file, *rosargs], output="screen")]
          if terrain.get("enabled") and robot["camera_enabled"] and runtime["semantic_segmentation"]["enabled"] and terrain.get("stairs_classes") else []),
        *([Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            remappings=[("cloud_in", policy["filtered_cloud"] if filter_cloud else policy["cloud"]), ("scan", policy["scan"])],
            parameters=[
                {
                    "target_frame": projection["target_frame"],
                    "transform_tolerance": projection["transform_tolerance_s"],
                    "min_height": -100.0 if filter_cloud else projection["min_height_m"],
                    "max_height": 100.0 if filter_cloud else projection["max_height_m"],
                    "range_min": projection["range_min_m"],
                    "range_max": projection["range_max_m"],
                    "use_inf": True,
                }
            ],
            output="screen",
        )] if robot["lidar_enabled"] else []),
        *([*camera_children,
            ExecuteProcess(cmd=[worker_python, "-m", "outdoor_nav.vision_filter", runtime_file, *rosargs], output="screen")]
          if robot["camera_enabled"] else []),
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
                *[RegisterEventHandler(
                    OnProcessExit(
                        target_action=child,
                        on_exit=[
                            EmitEvent(
                                event=Shutdown(reason="Outdoor service child exited")
                            )
                        ]
                    )
                ) for child in children if child not in camera_optional],
                *children,
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument("runtime"), OpaqueFunction(function=nodes)]
    )
