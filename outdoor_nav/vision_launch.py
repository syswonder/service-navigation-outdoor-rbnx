"""Original RGB-D geometry and optional segmentation, with deployment bindings."""
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def vision_nodes(runtime, python, rosargs):
    p, t = runtime["policy"], runtime["topic_remap"]
    terrain = runtime.get("terrain", {})
    slope = terrain.get("slope_filter_enabled", False) and terrain.get("camera_slope_enabled", True)
    angle = terrain.get("slope_angle_deg", 15.) if slope else 12.
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
              "/semantic/road_mask": t["camera_road_mask"],
              "/semantic/current_grid": t["camera_semantics"],
              "/semantic/local_grid": "/outdoor/vision/semantic_grid"}
    args = [v for a, b in remaps.items() for v in ("-r", a+":="+b)]
    depth = dict(allow_outdoor_navigation=True, planning_frame="odom", base_frame="base_link",
                 # RTAB-Map ground/obstacle segmentation replies in 0.36-0.82 s here while the
    # pairing slot accepts at most 0.3 s, so every frame expired; the in-process
    # geometry path already fits the ground plane (offset -0.42 in base_link).
    geometry_primary=True, depth_backend="legacy", alignment_verified=p["camera_frame_confirmed"],
                 geometry_expected_ground_z=p["camera_expected_ground_z_m"],
                 geometry_ground_z_tolerance=p["camera_ground_tolerance_m"],
                 max_ground_angle_deg=angle, max_range=p["camera_max_range_m"], min_height=.10, max_age=p["sensor_timeout_s"], pixel_stride=3)
    projection = dict(planning_frame="odom", robot_base_frame="base_link", alignment_verified=p["camera_frame_confirmed"],
                      slope_envelope=slope, slope_angle_deg=angle, ground_reference_z=terrain["ground_z_m"], forward_range=p["camera_max_range_m"], depth_max=p["camera_max_range_m"]+1., ground_limits_confirmed=True, ground_z_min=p["camera_ground_z_min_m"],
                      ground_z_max=p["camera_ground_z_max_m"], grid_alignment="planning",
                      origin_x=-6., origin_y=-6., width=240, height=240, resolution=.05,
                      pixel_stride=2, temporal_frames=1, opening=False, closing=False,
                      max_data_age=p["sensor_timeout_s"])
    import json
    segmentation = dict(config_json=json.dumps(runtime["semantic_segmentation"]),
                        rgb_topic=t["camera_rgb"], mask_topic=t["camera_road_mask"],
                        labels_topic=t["camera_labels"], confidence_topic=t["camera_confidence"],
                        status_topic=t["camera_segmentation_status"], overlay_topic=t["camera_overlay"],
                        max_image_age=p["sensor_timeout_s"])
    children = []
    optional = []
    geometry_enabled = p["camera_geometry_enabled"]
    node_parameters = [("depth", depth)] if geometry_enabled else []
    if runtime["semantic_segmentation"]["enabled"]:
        node_parameters += [("segmentation", segmentation), ("projection", projection)]
    for kind, parameters in node_parameters:
        path = Path(runtime["run_dir"]) / (kind+"_original.yaml")
        path.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
        action = ExecuteProcess(cmd=[python, "-m", "outdoor_nav.vision_nodes", kind,
            *rosargs, *args, "--params-file", str(path)], additional_env=env, output="screen")
        children.append(action)
        if geometry_enabled and kind in ("segmentation", "projection"):
            optional.append(action)
    if geometry_enabled and depth["depth_backend"] == "rtabmap":
        children.append(Node(package="rtabmap_util", executable="obstacles_detection", name="outdoor_rtabmap_obstacles",
            remappings=[("cloud", "/outdoor/vision/rtabmap/input"), ("ground", "/outdoor/vision/rtabmap/ground"),
                        ("obstacles", "/outdoor/vision/rtabmap/obstacles")], parameters=[{
                "frame_id": "base_link", "qos": 2, "Grid/NormalsSegmentation": "true", "Grid/MaxGroundAngle": str(angle),
                "Grid/MinGroundHeight": "-0.10", "Grid/MaxGroundHeight": "0.10", "Grid/MaxObstacleHeight": "1.7",
                "Grid/MinClusterSize": "5", "Grid/ClusterRadius": "0.10", "Grid/NormalK": "20",
                "Grid/CellSize": "0.04", "Grid/RangeMax": str(p["camera_max_range_m"]+1.), "Grid/PreVoxelFiltering": "true"}]))
    return children, optional
