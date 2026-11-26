# Recorded-gait navigation backport

Base: main fe738b6. No visual perception, terrain recognition state machine or decision-cycle planner is included.

- Keep continuous straight-road detours from 84ae33d; bends remain ordered.
- Share the adjacent-segment corner predicate between route progress and gait progress.
- Release the old controller path on localization recovery or epoch reset; keep ordered route progress.
- Use lidar acquisition time for freshness and ignore out-of-order frames.
- Clear a resolved command-publisher conflict without replaying cached velocity.
- Preserve configured controller goal tolerance and report the failed planning station.
- Separate chassis preparation eligibility from permission to move; actual gait feedback still gates velocity.

## Mandatory gait boundaries

`policy.gait_boundary_tolerance_m` defaults to 0.25 m. A station at which the recorded gait changes is retained through route simplification and cannot be skipped, shifted to a nearby free cell, or passed solely by an end-plane/corner predicate. The planning horizon ends at that station until the estimated robot position enters its tolerance. An outstanding gait boundary also prevents the final three-metre arrival rule from ending the task early.

Ordinary intermediate stations retain their existing pass-through conditions. The final destination tolerance remains 3 m in field deployment. The controller's tolerance is capped to half the boundary tolerance on routes with gait transitions, leaving room for the planner endpoint error. A required boundary with no feasible path causes a diagnostic stop rather than a shortcut.

The recorded gait scheduler still uses its existing one-metre pre/post stair interval margin and confirmed feedback. That early *gait preparation* does not permit cutting the stair-entry corner: the mandatory route checkpoint must still be visited. Turning within a recorded stair region retains main's temporary flat-gait rotation behavior.

These checks use estimated map coordinates, not physical foot-to-step measurements. Simulation validates route ordering and feedback, not physical climbing or safe human avoidance.
