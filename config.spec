# Instance config delivered through Driver INIT. Complete Nav2 behavior is deploy-owned.
# See docs/CONFIGURATION.md for migration and the effective parameter artifact.
specVersion: 1
description: Outdoor reference-route execution with independent lidar and original RGB-D perception.
required: [params_file]
additionalProperties: false
properties:
  params_file:
    type: string
    x-group: Nav2 parameters
    description: Complete deploy-owned Nav2 YAML. Relative paths resolve from RBNX_INVOCATION_CWD. Requires planner/controller/smoother and both costmaps. Topic tokens are bound without replacing the robot tuning.
  provider_ids:
    type: object
    x-group: Inputs
    additionalProperties: false
    properties:
      cloud: {type: string, x-provider: robonix/primitive/lidar/lidar3d}
      odom: {type: string, x-provider: robonix/service/localization/odom}
      localization: {type: string, x-provider: robonix/service/localization/status}
      chassis: {type: string, x-provider: robonix/primitive/chassis/gait_in}
      camera: {type: string, x-provider: robonix/primitive/camera/rgb}
    description: Atlas provider instance ids. odom defaults to localization. Disabled sensors are not discovered. Registered depth CameraInfo is explicitly bound; chassis is needed only for guarded gait switching.
  topic_remap:
    type: object
    x-group: Inputs
    additionalProperties: false
    description: Absolute ROS topics. Explicit values take priority over Atlas for cloud/odom/localization. Other roles bind wrapper endpoints; Nav2 consumes the matching topic tokens.
    properties:
      camera_rgb:
        type: string
        default: "/camera_gemini330/color/image_raw"
      camera_depth:
        type: string
        default: "/camera_gemini330/depth/image_raw"
      camera_depth_info:
        type: string
        default: "/camera_gemini330/depth/camera_info"
      camera_color_info:
        type: string
        default: "/camera_gemini330/color/camera_info"
      camera_obstacles:
        type: string
        default: "/outdoor/direct_nav/visual_obstacles"
        description: Original detector guarded observation endpoint; must remain /outdoor/direct_nav/visual_obstacles.
      camera_ground:
        type: string
        default: "/outdoor/direct_nav/observed_ground"
      camera_status:
        type: string
        default: "/outdoor/direct_nav/depth_status"
      camera_semantics:
        type: string
        default: "/outdoor/vision/current_grid"
      camera_labels: {type: string, default: /outdoor/vision/semantic_labels}
      camera_confidence: {type: string, default: /outdoor/vision/semantic_confidence}
      camera_road_mask: {type: string, default: /outdoor/vision/road_mask}
      camera_segmentation_status: {type: string, default: /semantic/segmentation_status}
      camera_overlay: {type: string, default: /semantic/debug_overlay}
      camera_costmap:
        type: string
        default: "/outdoor/vision/filtered_costmap"
      camera_scan:
        type: string
        default: "/outdoor/camera_scan"
      cloud:
        type: string
      scan:
        type: string
        default: "/outdoor/scan"
      odom:
        type: string
      localization:
        type: string
      chassis:
        type: string
        default: "/outdoor/chassis_status"
      tf:
        type: string
        default: "/outdoor/tf"
      tf_static:
        type: string
        default: "/tf_static"
  scan_projection:
    type: object
    x-group: Scan projection
    additionalProperties: false
    description: Lidar PointCloud2-to-LaserScan adapter only. Camera detection and stability filtering use their independent chain. All distances use metres, times use seconds.
    properties:
      enabled:
        type: boolean
        description: Defaults to policy.lidar_enabled and must match it for the supported 3D lidar input.
      target_frame: {type: string, default: base_link, enum: [base_link]}
      min_height_m: {type: number, default: -0.3}
      max_height_m: {type: number, default: 0.3}
      range_min_m: {type: number, default: 0.5, minimum: 0}
      range_max_m: {type: number, default: 8.0, exclusiveMinimum: 0}
      transform_tolerance_s: {type: number, default: 0.2, minimum: 0}
  semantic_segmentation:
    type: object
    x-group: Vision
    additionalProperties: false
    description: Optional replaceable semantic segmentation. Enabled backends require a weights file or directory and class table; see README.
    properties:
      enabled: {type: boolean, default: false}
      backend: {type: string, default: "", description: "Importable module:factory. No default model selection; optional SegFormer adapter available."}
      weights: {type: string, description: "Required when enabled; weights file or directory, relative to deploy manifest directory. Format validated by adapter."}
      device: {type: string, default: auto}
      confidence_threshold: {type: number, default: 0.7, minimum: 0, maximum: 1}
      options: {type: object, additionalProperties: true}
      classes: {type: object, default: {}, additionalProperties: {type: string}, description: "Model IDs 0..65534 to semantic names; 65535 is reserved for unknown."}
      road_classes: {type: array, default: [], uniqueItems: true, items: {type: string}, description: "Names from classes used for road soft costs. Does not override geometric obstacles."}
  policy:
    type: object
    x-group: Execution and observation
    additionalProperties: false
    description: Outdoor task acceptance, observation freshness, original camera calibration/filtering and commissioning confirmations. Nav2 tuning belongs in params_file.
    properties:
      lidar_enabled:
        type: boolean
        default: true
        description: "Use the independent lidar obstacle layer and its health gate."
      camera_enabled:
        type: boolean
        default: false
        description: "Use the original RGB-D/RTAB-Map chain followed by stability filtering and its own costmap layer."
      camera_frame_confirmed:
        type: boolean
        default: false
        description: "Measured depth registration and camera extrinsics confirmation; false does not manufacture free space."
      camera_expected_ground_z_m:
        type: number
        default: -0.18
        description: "Expected ground height in the original base frame, metres; allowed range -0.6 to -0.05."
      camera_ground_tolerance_m:
        type: number
        default: 0.05
        description: "Original ground calibration tolerance, metres; allowed range 0.005 to 0.1."
      camera_ground_z_min_m:
        type: number
        default: -0.45
        description: "Lower original semantic ground projection height in metres."
      camera_ground_z_max_m:
        type: number
        default: 0.15
        description: "Upper original semantic ground projection height in metres."
      camera_static_min_hits:
        type: integer
        default: 4
        description: "Distinct source-stamped observations required for a positive obstacle; integer at least 2."
      camera_static_dwell_s:
        type: number
        default: 0.6
        description: "Minimum positive-obstacle stable duration in seconds."
      camera_static_gap_s:
        type: number
        default: 0.35
        description: "Maximum gap between matching observations in seconds."
      camera_static_cell_m:
        type: number
        default: 0.15
        description: "Spatial association cell size in odom, metres."
      camera_static_drift_m:
        type: number
        default: 0.12
        description: "Maximum drift from the track anchor, metres. Negative obstacles retain the original handling."
      sensor_timeout_s:
        type: number
        default: 1.0
        description: "Worker and guard maximum sensor feedback age, seconds. Set camera_layer.max_age_s separately in the Nav2 YAML."
      command_timeout_s:
        type: number
        default: 0.5
        description: "Independent final command guard timeout, seconds; does not rewrite smoother.velocity_timeout."
      navigation_recovery_timeout_s:
        type: number
        default: 15.0
        description: "Maximum wait for navigation readiness recovery during a route, seconds."
      fixed_arrival_m:
        type: number
        default: 3.0
        description: "GNSS fixed-grade route arrival tolerance, metres; distinct from the Nav2 local goal checker."
      float_arrival_m:
        type: number
        default: 3.0
        description: "GNSS float-grade route arrival tolerance, metres; at least fixed_arrival_m."
      max_speed_mps:
        type: number
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use controller_server.FollowPath and velocity_smoother speed limits. Guard speed limits are otherwise derived from the effective YAML."
      max_angular_speed_rps:
        type: number
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use controller_server.FollowPath and velocity_smoother yaw limits. Guard speed limits are otherwise derived from the effective YAML."
      footprint_radius_m:
        type: number
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use both costmaps robot_radius or footprint. Guard speed limits are otherwise derived from the effective YAML."
      cloud_min_height_m:
        type: number
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use scan_projection.min_height_m. Guard speed limits are otherwise derived from the effective YAML."
      cloud_max_height_m:
        type: number
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use scan_projection.max_height_m. Guard speed limits are otherwise derived from the effective YAML."
      scan_range_min_m:
        type: number
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use scan_projection.range_min_m. Guard speed limits are otherwise derived from the effective YAML."
      planner_patience_s:
        type: number
        default: 60.0
        description: "Reference-route executor planning retry budget, seconds, from 0 to 300."
      controller_patience_s:
        type: number
        default: 60.0
        description: "Reference-route executor controller retry budget, seconds, greater than 0 and at most 300."
      stopped_linear_mps:
        type: number
        default: 0.03
        description: "Measured linear speed threshold for stop confirmation, metres per second. Does not rewrite DWB."
      stopped_angular_rps:
        type: number
        default: 0.03
        description: "Measured angular speed threshold for stop confirmation, radians per second."
      stop_hold_s:
        type: number
        default: 1.0
        description: "Continuous measured standstill duration required for stopping, seconds."
      gait_switch_timeout_s:
        type: number
        default: 10.0
        description: "Fresh chassis feedback wait budget during a marked gait switch, seconds."
      transform_tolerance_s:
        type: number
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use costmap/controller transform_tolerance and scan_projection.transform_tolerance_s. Guard speed limits are otherwise derived from the effective YAML."
      inflation_padding_m:
        type: number
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use both costmaps inflation_layer.inflation_radius. Guard speed limits are otherwise derived from the effective YAML."
      inflation_cost_scaling:
        type: number
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use both costmaps inflation_layer.cost_scaling_factor. Guard speed limits are otherwise derived from the effective YAML."
      planner_tolerance_m:
        type: number
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use planner_server.GridBased.tolerance. Guard speed limits are otherwise derived from the effective YAML."
      local_track_unknown:
        type: boolean
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use local_costmap.track_unknown_space. Guard speed limits are otherwise derived from the effective YAML."
      obstacle_layer_enabled:
        type: boolean
        deprecated: true
        description: "Legacy explicit override; emits migration warnings. Use enabled flags on the selected costmap layers. Guard speed limits are otherwise derived from the effective YAML."
      allow_float_during_fixed_task:
        type: boolean
        default: true
        description: "Allow degraded float localization during a task that began fixed, retaining its acceptance rules."
      motion_enabled:
        type: boolean
        default: false
        description: "Enable motion after commissioning. Default false; does not reconnect the configured velocity sink."
      require_chassis_ready:
        type: boolean
        default: true
        description: "Require fresh external chassis ready JSON in addition to localization and odometry."
      calibrated:
        type: boolean
        default: false
        description: "Outdoor localization calibration has been independently verified."
      antenna_offset_confirmed:
        type: boolean
        default: false
        description: "GNSS antenna offset has been measured and verified."
      footprint_confirmed:
        type: boolean
        default: false
        description: "Deployment-owned robot collision footprint has been verified."
      cloud_frame_confirmed:
        type: boolean
        default: false
        description: "Lidar frame and mounting have been verified."
      feedback_confirmed:
        type: boolean
        default: false
        description: "Fresh velocity feedback and ownership have been verified."
      odometry_confirmed:
        type: boolean
        default: false
        description: "External odometry has been verified for external_odom mode."
      localization_mode:
        type: string
        default: "rtk_imu"
        description: "Feedback mode accepted by the outdoor execution gate."
        enum: [rtk_imu, external_odom]
      heading_source:
        type: string
        default: "wit_imu"
        description: "Configured heading source accepted by the outdoor execution gate."
        enum: [wit_imu, configured]
  velocity_output_topic:
    type: string
    x-group: Velocity output
    default: /robonix/nomotion/outdoor_cmd_vel
    description: Absolute sole output of the final command guard. Empty, malformed and feedback-loop topics fail initialization. Never defaults to a physical chassis topic.
  gait_switching_enabled:
    type: boolean
    x-group: Execution
    default: false
    description: Stop and wait for fresh feedback at marked route gait changes. Requires deploy-owned StairsFollowPath when enabled; the wrapper never inserts that controller.
  gait_output_topic:
    type: string
    x-group: Execution
    description: Guarded UInt32 output; if absent with gait switching enabled, resolve the selected chassis gait_in endpoint. Otherwise defaults to /robonix/nomotion/outdoor_gait.
  global_grid:
    type: object
    x-group: Deprecated
    deprecated: true
    description: Legacy explicit global costmap geometry override with warning. New deployments configure global_costmap in params_file.
    required: [origin, width, height, resolution]
    additionalProperties: false
    properties:
      origin: {type: array, items: {type: number}, minItems: 2, maxItems: 2}
      width: {type: number, exclusiveMinimum: 0}
      height: {type: number, exclusiveMinimum: 0}
      resolution: {type: number, exclusiveMinimum: 0}
      rolling: {type: boolean, default: false}
  run_dir:
    type: string
    x-group: Diagnostics
    description: Owned runtime parameters, worker log and PID directory. Relative to deploy manifest. Defaults to package rbnx-build/runtime.
  http_port:
    type: integer
    x-group: Startup
    default: 19975
    minimum: 1024
    maximum: 65535
    description: Localhost-only internal worker control transport behind Robonix.
  action_wait_s:
    type: number
    x-group: Startup
    default: 45.0
    exclusiveMinimum: 0
    description: Worker initialization status wait, seconds. Sensor/Nav2 readiness continues to gate each task.
  worker_python:
    type: string
    x-group: Startup
    description: Optional ROS-compatible worker interpreter. Defaults to the bridge interpreter.
