# Implementation and study session log — 2026-09-11

This log records commands, decisions, failures, validation evidence, and known
limitations. It is engineering evidence, not experimental evidence.

## Baseline

- Starting commit: `d96ba27` (`Implement morphology-aware modular robot navigation platform`).
- Starting worktree: clean.
- Host tests: `python3 -m pytest -q` — 16 passed in 2.33 s.
- Host ROS build could not run because ROS is installed only in the development
  container (`/opt/ros/jazzy/setup.bash` is absent on the host).
- Container build: all nine packages passed `colcon build --symlink-install`.
- Container package tests ran without errors. `colcon test-result` reported zero
  aggregated tests because the five Python smoke tests are emitted by package
  setup test hooks rather than registered as colcon result files.

## Decisions

- Confirmatory experiments use only `compact_diff` and `narrow_tandem`, the two
  catalog morphologies whose differential locomotion matches the current fixed
  pod wheel mechanics.
- Ackermann, omni, crawler, articulated, and stacked configurations remain
  future concepts until their mechanics are implemented and validated.
- Nav2 costmap footprint input is `geometry_msgs/msg/Polygon` on installed Jazzy;
  `PolygonStamped` is the costmap output type.
- Raw mission logs will be immutable JSON records. Derived tables and figures
  will be written to a separate analysis directory.

## Outstanding limitations

- No mission trial from the current session has yet passed the live-SLAM gate.
- Existing `edge_observations.csv` is synthetic scaffolding and is excluded from
  all study analysis.
- Gazebo latch acknowledgement does not establish physical connector capture.

## Continued implementation

- Added sensor-derived relative localization using topology-anchored wheel
  odometry and four visibility-limited Gazebo logical cameras. The executor no
  longer subscribes to `/module_pose`; that topic remains evaluation-only.
- Added a conservative time-parameterized 3D transition validator with
  environment/self collision, support, sensing, covariance, speed, connector
  reach, and latch checks. Planner-edge integration remains pending.
- Added an immutable confirmatory design at
  `studies/confirmatory/design.json`: 432 trials, design hash
  `e9115d543af0969db7825398f7e2691361bdb536530d93e49db4528f21c51cf5`.
- Corrected the stratified layout bootstrap to retain the multiplicity of
  resampled layouts. Added method-level completion intervals, secondary
  deadline-time contrasts, family-effect diagnostics, per-method calibration,
  prospective power simulation, derived tables, and SVG figures.
- Host verification after these changes: 51 tests passed. All nine ROS packages
  built successfully after the conventional wheel-frame SDF refactor.
- Bullet Featherstone was found to report rotating pod odometry while the
  detached pod's physical yaw remained effectively fixed. Classic Bullet was
  rejected because Gazebo reports that it lacks `AttachFixedJointFeature`.
  DART supports both required behaviors, but repeated attachments initially
  caused duplicate internal wheel/joint names and an ODE collision crash.
- The simulator now generates six mechanically identical pod SDFs with unique
  internal link, collision, and joint names. This removes the DART skeleton-name
  collisions during attachment. The supported narrow morphology was redesigned
  as a ground-level staggered two-track arrangement whose footprint includes
  the actual wheel envelopes; the prior 0.34 m-wide layout intersected the core
  and implicitly depended on unsupported stacking.
- A sensor-driven `compact_to_narrow` run completed in 71.356 s with all six
  observed latch states committed. The immediate `narrow_to_compact` run failed
  safely at `pod_0` relocation waypoint 1/3 after 51 s. Repeated round-trip
  qualification therefore remains open; neither run is confirmatory evidence.
