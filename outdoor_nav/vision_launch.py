"""Original RGB-D/RTAB-Map/PIDNet nodes, with deployment-only bindings."""
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def vision_nodes(runtime, python, rosargs):
    p, t = runtime["policy"], runtime["topic_remap"]
    from pathlib import Path
    import yaml
    import os
    vendor = Path(__file__).resolve().parents[1] / "vendor"
    env = {"PYTHONPATH": str(vendor) + os.pathsep + os.environ.get("PYTHONPATH", "")}
    # Keep original observation names to satisfy the original node's live-domain guard.
    remaps = {"/camera_gemini330/color/image_raw": t["camera_rgb"],
              "/camera_gemini330/depth/image_raw": t["camera_depth"],
              "/camera_gemini330/depth/camera_info": t["camera_depth_info"],
              "/camera_gemini330/color/camera_info": t["camera_color_info"],
              "/direct_nav/visual_obstacles": t["camera_obstacles"],
              "/direct_nav/observed_ground": t["camera_ground"],
              "/direct_nav/depth_status": t["camera_status"],
              "/direct_nav/rtabmap/input": "/outdoor/vision/rtabmap/input",
              "/direct_nav/rtabmap/ground": "/outdoor/vision/rtabmap/ground",
              "/direct_nav/rtabmap/obstacles": "/outdoor/vision/rtabmap/obstacles",
              "/semantic/road_mask": "/outdoor/vision/road_mask",
              "/semantic/current_grid": t["camera_semantics"],
              "/semantic/local_grid": "/outdoor/vision/semantic_grid"}
    args = [v for a, b in remaps.items() for v in ("-r", a+":="+b)]
    depth = dict(allow_outdoor_navigation=True, planning_frame="odom", base_frame="base_link",
                 geometry_primary=True, depth_backend="rtabmap", alignment_verified=p["camera_frame_confirmed"],
                 geometry_expected_ground_z=p["camera_expected_ground_z_m"],
                 geometry_ground_z_tolerance=p["camera_ground_tolerance_m"],
                 min_height=.10, max_age=p["sensor_timeout_s"], pixel_stride=3)
    projection = dict(planning_frame="odom", robot_base_frame="base_link", alignment_verified=p["camera_frame_confirmed"],
                      ground_limits_confirmed=True, ground_z_min=p["camera_ground_z_min_m"],
                      ground_z_max=p["camera_ground_z_max_m"], grid_alignment="planning",
                      origin_x=-6., origin_y=-6., width=240, height=240, resolution=.05,
                      pixel_stride=2, temporal_frames=1, opening=False, closing=False,
                      max_data_age=p["sensor_timeout_s"])
    pidnet = dict(pidnet_weights=runtime.get("pidnet_weights", ""), mode="pidnet", test_mode=False,
                  device=runtime.get("pidnet_device", "auto"),
                  traversable_train_ids=[0, 1], confidence_threshold=.7, max_image_age=p["sensor_timeout_s"])
    if p["pidnet_enabled"] and not Path(pidnet["pidnet_weights"]).is_file():
        raise ValueError("pidnet_weights must reference the original PIDNet-M checkpoint")
    children = []
    optional = []
    node_parameters = [("depth", depth)]
    if p["pidnet_enabled"]:
        node_parameters += [("pidnet", pidnet), ("projection", projection)]
    for kind, parameters in node_parameters:
        path = Path(runtime["run_dir"]) / (kind+"_original.yaml")
        path.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
        action = ExecuteProcess(cmd=[python, "-m", "outdoor_nav.vision_nodes", kind,
            *rosargs, *args, "--params-file", str(path)], additional_env=env, output="screen")
        children.append(action)
        if kind in ("pidnet", "projection"):
            optional.append(action)
    children.append(Node(package="rtabmap_util", executable="obstacles_detection", name="outdoor_rtabmap_obstacles",
        remappings=[("cloud", "/outdoor/vision/rtabmap/input"), ("ground", "/outdoor/vision/rtabmap/ground"),
                    ("obstacles", "/outdoor/vision/rtabmap/obstacles")], parameters=[{
            "frame_id": "base_link", "qos": 2, "Grid/NormalsSegmentation": "true", "Grid/MaxGroundAngle": "12",
            "Grid/MinGroundHeight": "-0.10", "Grid/MaxGroundHeight": "0.10", "Grid/MaxObstacleHeight": "1.7",
            "Grid/MinClusterSize": "5", "Grid/ClusterRadius": "0.10", "Grid/NormalK": "20",
            "Grid/CellSize": "0.04", "Grid/RangeMax": "6.0", "Grid/PreVoxelFiltering": "true"}]))
    return children, optional
