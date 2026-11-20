# Execution and recovery ownership

## Configuration is not a recovery strategy

The executor separates three questions:

1. **Can a task be accepted?** The map identity, localization, enabled observation sources, output ownership and chassis mode must be valid. New tasks require fixed/float RTK; an already accepted task may use valid bounded inertial continuation.
2. **Can a gait request be sent?** Fresh chassis state 17, navigation usage mode 1 and an armed gait output are required. Being in the requested gait is deliberately not a prerequisite for requesting it.
3. **Can velocity be released?** The current task must own the controller, the requested gait must be confirmed by fresh feedback, sensor data must remain valid, and the final command watchdog must receive fresh commands.

A chassis reporting its ordinary gait can therefore prepare the navigation gait without disabling `require_chassis_ready`. Missing feedback, the wrong control mode, a disarmed output or an ownership conflict still prohibits motion. No navigation task automatically stands a physical robot up or changes its usage mode.

## Planner and terrain are independent

`decision_cycle.enabled` chooses planning cadence and snapshot delivery. `terrain.enabled` chooses terrain supervision. Neither switch disables the shared gait handshake.

- Route mode uses recorded, ordered gait sections, including main's temporary flat-gait rotation on stair sections.
- Terrain with `can_climb_stairs=false` ignores recorded stair commands and uses a confirmed flat navigation gait. Terrain can still prohibit motion on an unsupported detected stair.
- Terrain with `can_climb_stairs=true` owns its semantic/alignment/gait phases. The mode needs an explicitly configured semantic source and hardware codes. Missing required semantics remains unknown; it is not permission to climb.

Ordinary planning and gait scheduling use the same ordered corner-transition geometry. An obstacle detour onto the immediately following edge may advance progress without touching a recorded vertex. Neither component searches a later parallel arm of the road to fabricate progress.

## Time ownership

- `sensor_timeout_s`: validity of actual obstacle observations. Planning cadence does not silently overwrite it.
- `localization_timeout_s`: one budget for pose, localization status and velocity. `pose_timeout_s` remains a migration alias; conflicting values are rejected.
- `command_timeout_s`: independent final actuator-command watchdog. Increasing a localization budget does not increase command lifetime.
- `navigation_recovery_timeout_s`: how long a stopped task may await recovery. Gait preparation inherits this budget unless an old deployment explicitly overrides its migration field. Actual gait confirmation is a separate bounded handshake.

The TF listener has its own node/thread and depth-one dynamic queue, as on main. This fixes reception backlog rather than accepting arbitrarily old transforms. Cycle planning keeps a bounded scan history and selects the freshest observation whose acquisition-time TF is available, without blocking reception, relabeling frames or changing source timestamps. If no fresh matched pair exists, motion is held and the missing dependency is reported.

Transient data loss produces zero output and a bounded recovery attempt. Ownership/motion-permission/clock faults still terminate the task. Removal of a competing command publisher clears live conflict readiness but first invalidates the cached command; it does not replay motion or restart a completed task.

## Perception and planning

The lidar's main distance-graded height filter and the optional terrain envelope are independent of the planning mode. Explicit `scan_projection.min_height_m/max_height_m` retain the feature branch's fixed body-frame band; otherwise main's ground-relative grading is used, without another user-facing mode switch. The ground reference comes only from the machine-local file; the imported legacy lidar-height view is derived from that value, not a second calibration.

`__ROBONIX_SCAN_TOPIC__` binds to the gated `obstacle_scan` endpoint. Raw `scan` remains available for sensor health and geometry even when route-mode stair policy withholds obstacle marking. Camera observations never clear lidar obstacles.

RGB-D geometry uses an explicit `camera_geometry_backend` (`legacy` in-process geometry or `rtabmap`). Both paths validate their observations; neither silently substitutes the configured ground plane for missing depth. An enabled camera remains a required observation source. A lidar-only deployment must select that mode explicitly rather than silently losing depth coverage during a task.

Nav2 parameters belong to `params_file`. Do not also specify a legacy `global_grid` in a new deployment. Ordinary execution uses a bounded route horizon, not a single request to the distant final destination; cycle mode uses a bounded odom snapshot. A large survey map does not by itself require a map-sized sensor costmap. Planner failures report the failed intermediate point, action status and path size; timeouts and absence of a path are different failures.

## Verification boundary

Regression tests cover merge behavior, all four planner/terrain switch combinations, preparation versus velocity gating, timestamp pairing, ordered detours, stale data, ownership and recovery. The workspace's `simulation` runs real Scene, localization and Nav2 against a separate simulated chassis/RTK/IMU domain, including standing-to-navigation-gait preparation, arrival, cancellation and IMU loss/reconnection.

Simulation does not commission physical stairs, depth under sunlight, camera extrinsics, magnetic heading or real obstacle clearance. A passing indoor run is required before another outdoor test, not a substitute for one.
