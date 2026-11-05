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
  topic_remap:
    type: object
    description: Absolute ROS topics; explicit values override Atlas. Roles cloud, scan, odom, localization, chassis, tf, tf_static. localization is resolved from the selected localization service unless explicitly bound.
  policy:
    type: object
    description: See outdoor_nav/configuration.py DEFAULTS; same field execution thresholds, confirmed calibration/feedback and motion gates. No raw sensor configuration.
  velocity_output_topic:
    type: string
    default: /robonix/nomotion/outdoor_cmd_vel
  global_grid:
    type: object
    description: Sensor-only map extent; origin in metres, width/height in metres, resolution in metres/cell. No static mask.
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
