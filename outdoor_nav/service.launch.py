"""Pure deployment launch: no localizer, drivers, static map or business server."""

import json
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
        ("/outdoor/camera_scan", policy["camera_scan"]),
        ("/cmd_vel", runtime["velocity_output_topic"]),
    ]
    rosargs = ["--ros-args"] + [arg for a, b in remaps for arg in ("-r", a + ":=" + b)]
    projection = runtime["scan_projection"]
    camera_children, camera_optional = vision_nodes(runtime, worker_python, rosargs) if robot["camera_enabled"] else ([], [])
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
                "-p",
                "robot_config:=" + runtime_file,
            ],
            output="screen",
        ),
        *([Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            remappings=[("cloud_in", policy["cloud"]), ("scan", policy["scan"])],
            parameters=[
                {
                    "target_frame": projection["target_frame"],
                    "transform_tolerance": projection["transform_tolerance_s"],
                    "min_height": projection["min_height_m"],
                    "max_height": projection["max_height_m"],
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
