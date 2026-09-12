# Development session log — 2026-09-12

## Mission-runner and inference continuation

- Exported complete confirmatory SDF worlds and manifests and added launch-time
  world selection.
- Added a resumable full-stack mission runner with readiness gates, immutable
  records, simulated deadlines, wall watchdogs, and rolling-work proxy labeling.
- Diagnosed four preserved debug smoke trials: clock QoS, premature goal send,
  goal outside the initial SLAM grid, and finally an unobserved distant doorway.
  None is pilot or confirmatory evidence.
- Added navigation-result telemetry for cumulative planning latency, expanded
  states, map/topology/sensing revisions, a deterministic route signature, and
  planned transition sites.
- Changed pilot-scale paired randomization inference to exact sign-flip
  enumeration at up to 20 layouts; larger frozen contrasts remain reproducible
  seeded Monte Carlo tests and report the inference mode.
- Next experimental fix: provide generated prior occupancy maps while retaining
  sensor-based localization, then add evaluator-only truth metrics and a
  sensor-derived location-dependent 3D/observability context.

## Scope

Continued the self-mobile compact/narrow study toward the bounded contribution:
joint route and morphology selection conditioned on transition feasibility,
sensing uncertainty, and time/energy/failure-risk cost.

## Decisions and changes

- Kept `compact_diff` and `narrow_tandem` as the only confirmatory
  morphologies. Unsupported Ackermann, omni, crawler, articulated, and stacking
  catalog entries remain future concepts.
- Added catalog parsing and validation for pod poses, transition waypoints,
  module sizes, and compound collision boxes.
- Found that the prior narrow track and coarse pod box produced a planned
  wheel/body sweep collision. Changed the narrow track from ±0.08 m to ±0.10 m,
  expanded its footprint to ±0.19 m, and modeled the pod body and two wheel
  bounds separately. The nominal transition now passes its intrinsic
  self-collision check.
- Added time-parameterized sequential pod trajectories transformed from the
  core frame into each candidate transition state's world pose.
- Added structured transition decisions for planar clearance, target footprint,
  3D environment/self collision, support, relative velocity, reach, latch,
  connector visibility, and covariance.
- Implemented all four frozen methods. `route_first_adaptation` freezes a
  morphology-agnostic spatial route before assigning morphologies; the three
  coupled methods share hybrid A* and differ only by their transition policy.
- Bound plans to method, map, observed topology, and discrete sensing-signature
  revisions. Map revisions change only when map content/metadata changes.
  Navigation requests a fresh plan after successful reconfiguration or a
  relevant topology/sensing change.
- Replaced occupied-cell-center footprint testing with polygon/cell
  intersection so doorway corner clipping is rejected.
- Added deterministic held-out scenario generation for reconfiguration
  workspace, docking observability, combined constraints, and three neutral
  layouts per 12-layout family.
- Added stable plan signatures and serializable transition audits to terminal
  records. The analysis reports rejection taxonomies and paired route-decision
  disagreement as descriptive manipulation checks.

## Validation and observed failures

- Initial compound-box validation rejected `compact_to_narrow` because the
  earlier ±0.08 m track overlapped opposing wheel envelopes. The corrected
  ±0.10 m track passes.
- A first conservative cell-intersection change invalidated a four-cell test
  doorway and an old intermediate-rotation fixture. The doorway was corrected
  to a physically sufficient 0.5 m opening and the fixture was moved to a cell
  that intersects only the swept intermediate orientation.
- Planner-only golden run, layout seed 8:
  - workspace: geometry chose cell `(20,18)`; 3D feasibility chose `(20,11)`;
  - observability: feasibility chose `(20,18)`; sensing-aware chose `(20,23)`;
  - combined: feasibility chose `(20,11)`; sensing-aware chose `(20,24)`.
- These runs do not count as pilot or confirmatory missions.
- Complete host suite: 67 passed in 70.00 s.
- Fresh DART engineering run on the revised narrow geometry:
  `compact_to_narrow` passed in 70.118 s and `narrow_to_compact` passed in
  84.081 s. Both clients reported observed latch topology verified. The stack
  was stopped after the round trip. This is qualification evidence, not pilot
  or confirmatory data.

## Commands

```bash
python3 -m pytest -q
docker exec morphology_navigation_dev bash -lc \
  'cd /home/roboboat/morphology_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
docker exec morphology_navigation_dev bash -lc \
  'cd /home/roboboat/morphology_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon test --event-handlers console_direct+ && colcon test-result --verbose'
```

## Outstanding limitations

- The changed narrow geometry has one successful DART round trip; 19 further
  consecutive successes are required by the qualification gate.
- ROS currently applies the latest pod sensing signature to future candidate
  sites. The scenario framework supplies location-dependent predictions, but a
  perceived online observability field is still required.
- ROS does not yet populate the planner's 3D environment boxes from perceived
  world geometry.
- No unattended ROS/Gazebo mission runner or disjoint pilot exists. Frozen
  confirmatory status remains 0 of 432 terminal records.
- Logical cameras, magnetic capture, and latching remain idealized simulation
  mechanisms.
