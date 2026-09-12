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
