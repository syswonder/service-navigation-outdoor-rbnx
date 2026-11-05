---
description: Follow an external map-frame reference route with live obstacle avoidance and confirmed stopping.
---
# Outdoor navigation execution

Provides `robonix/service/navigation_outdoor/{follow_reference,status,cancel,stop}` over gRPC/MCP and `costmap` over ROS2. Driver explicitly declares the namespace lifecycle contract for SDK compatibility; no second Driver is declared.

`follow_reference(request_json)` accepts a JSON object with `frame_id: map`, finite `reference_path` points, `goal`, and a 32-character lowercase hex `request_id`. Optional `arrival_tolerance_m` and `localization_grade` override task policy. The final reference point must equal the exact goal; yaw is optional. Reuse request_id only for an identical execution. A new execution cancels its predecessor and waits for its measured stop before owning motion.

Poll `status(run_id)` after acceptance. `succeeded` means final goal and measured standstill passed; intermediate reference points are not stopping goals. `cancel` acceptance is not stop confirmation. `stop` deactivates and waits for fresh measured standstill; Driver activation is required to resume.

Consumes front lidar `robonix/primitive/lidar/lidar3d` and a deployment-selected robonix/service/localization/odom provider. The latter can carry RTK Doppler/WIT-generated Odometry; manufacturer `/ODOM` is explicitly rejected. Localization readiness is discovered from the same localization service; TF and optional chassis readiness are bound by deployment config. No static occupancy map is required. Sensor and feedback streams remain ROS2, not MCP.

No business road planner, scene, UI, raw GNSS/IMU driver, lidar driver or chassis activation is owned by this service. Only the bound output topic can carry velocity; default is `/robonix/nomotion/outdoor_cmd_vel`. No reverse/lateral movement; only front lidar is supported. The current near-range filter is compatibility behavior, not a validated body/leg filter. Pit/step/rear sensing is not implemented.
