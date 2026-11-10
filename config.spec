specVersion: 1
description: Outdoor Nav2 reference execution with external sensors and localization; default output is non-motion.
required: [params_file]
properties:
  params_file:
    type: string
    description: Deployment-owned Nav2 YAML, resolved relative to RBNX_INVOCATION_CWD.
  provider_ids:
    type: object
    properties:
      cloud:
        type: string
        x-provider: robonix/primitive/lidar/lidar3d
      odom:
        type: string
        x-provider: robonix/service/localization/odom
      localization:
        type: string
        x-provider: robonix/service/localization/status
      chassis:
        type: string
        x-provider: robonix/primitive/chassis/gait_in
  topic_remap:
    type: object
    description: Absolute ROS topics; explicit values override Atlas. Roles cloud, scan, odom, localization, chassis, tf, tf_static. localization is resolved from the selected localization service unless explicitly bound.
  policy:
    type: object
    description: See outdoor_nav/configuration.py DEFAULTS; same field execution thresholds, confirmed calibration/feedback and motion gates. No raw sensor configuration.
  velocity_output_topic:
    type: string
    default: /robonix/nomotion/outdoor_cmd_vel
  cloud_min_height_near_m:
    type: number
    default: 0.2
    description: Minimum point height at zero horizontal distance; grows with range.
  cloud_min_height_far_m:
    type: number
    default: 0.5
    description: Minimum point height reached at cloud_height_ramp_m and beyond.
  cloud_height_ramp_m:
    type: number
    default: 8.0
    description: Horizontal distance over which the minimum height rises from near to far.
  stairs_disable_obstacle_layer:
    type: boolean
    default: true
    description: Publish an infinite scan, clearing lidar marks, while a stair gait is requested.
  gait_switching_enabled:
    type: boolean
    default: false
    description: Stop before gait changes and wait for fresh feedback; policy.gait_switch_timeout_s defaults to 10 seconds. Stairs use reference tracking without local avoidance.
  gait_output_topic:
    type: string
    default: /robonix/nomotion/outdoor_gait
    description: UInt32 request topic; when enabled and omitted, resolve the selected chassis gait_in capability.
  global_grid:
    type: object
    description: Sensor-only map extent; origin, width/height and resolution are in metres. Optional rolling keeps the global sensor window around the robot. No static mask.
  run_dir:
    type: string
    description: Owned runtime files, PID, journal and logs.
  http_port:
    type: integer
    default: 19975
    description: Localhost-only worker control transport behind Robonix, not a robot web UI.
  action_wait_s:
    type: number
    default: 45
    description: Maximum wait for worker status during process initialization. Sensor and Nav2 readiness gate tasks separately.
