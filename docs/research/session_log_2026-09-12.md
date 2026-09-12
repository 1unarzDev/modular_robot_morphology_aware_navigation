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
- Implemented deterministic PGM/YAML prior-map export and AMCL launch with the
  scenario's frozen initial pose. The map is identical across paired methods;
  localization still consumes lidar and odometry rather than simulator pose.
- Preserved three further debug trials. The first exposed ROS parameter-parser
  rejection of aliases in the shared Nav2 YAML; localization now has a separate
  anchor-free parameter file. The second exposed volatile subscribers missing
  the latched map; planner and observer now request transient-local reliable
  QoS. The third planned and began execution: 107 expanded states, 1.234 s
  cumulative planning time, a compact-to-narrow site at approximately
  `(2.05, 1.85)`, and 107.6 J of the declared rolling-work proxy. Nav2 then
  reported no progress after repeated collision-monitor approach limiting.
  A subsequent fixed-route replan failure previously obscured that controller
  failure; terminal classification now retains the initiating execution fault.
- Added 0.5 s odometry and per-pod command histories to terminal records. A
  repeat debug run confirmed 83/123 samples had nonzero pod commands and the
  chassis moved 1.739 m, but it drifted from approximately `(0.65, 1.85)` to
  `(1.98, 2.97)` instead of reaching the planned `(2.05, 1.85)` transition
  site. The controller then stopped near the upper wall. This rejects command
  starvation as the main explanation and makes assembled path tracking the
  next diagnosis target.
- Added the complete planned route, signed body and pod commands, odometry yaw,
  and map-frame localization history to terminal records. A successful repeat
  confirmed the planned route is a straight 45-pose line at `y=1.85`, while
  Nav2 issued persistently positive yaw commands and the chassis curved toward
  the upper wall. This localizes the defect to controller feedback/frame or
  angular-sign behavior rather than hybrid-route geometry.
- Staggered Nav2 startup three seconds behind localization. One subsequent
  trace attempt still received no odometry/TF despite map server and AMCL
  reaching active state; it is retained as a debug infrastructure failure and
  shows that startup health must explicitly require fresh odometry and scan.

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

## Navigation safety, statistical framework, and RPP reruns

- Added `READY` gates before planning and every segment. A reconfiguration that
  leaves `RECOVERY_REQUIRED` now aborts immediately instead of being replanned.
- Added executor failure context: phase, active pod, relative pose, covariance,
  connector visibility, sensing sources, and latest latch observation.
- Added fresh `/scan` and `/odom` requirements to mission readiness and distinct
  classification for docking and unsafe-topology outcomes.
- Qualified assembled REP-103 motion signs: positive/negative yaw and straight
  travel all passed in `results/debug/motion_qualification4/result.json`.
- Moved confirmatory compact/narrow path following from MPPI to regulated pure
  pursuit. RPP reaches the transition site reliably in current engineering runs.
- Rebuilt all nine Jazzy packages and passed their package smoke tests.
- Ran two dirty-worktree engineering cases for combined-constraints layout 00.
  Both `feasibility_coupled` and `route_first_adaptation` completed one
  compact-to-narrow transition and returned `READY`, so they did not exercise
  the unsafe-state branch. Both then failed immediately on the post-transition
  traverse because regulated pure pursuit predicted a collision. Each record
  correctly contains one reconfiguration attempt; rapid retries were traversal
  controller retries, not repeated transition attempts. These are debug records.
- Expanded the statistical plan with the layout-balanced estimand, layout as the
  independent unit, sensitivity-model freeze, calibration coverage, and precise
  evaluator-only clearance/localization definitions.

Current blocker: synchronize Nav2's post-transition footprint and local costmap
with the new morphology, clear stale attached-body obstacles if present, and
prove that the narrow morphology can depart the transition site. Then inject a
partial transition failure to directly qualify the new unsafe-state gate.
