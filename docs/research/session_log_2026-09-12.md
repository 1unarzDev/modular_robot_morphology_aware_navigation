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

## Costmap footprint synchronization diagnosis

- Added an explicit post-transition barrier in `HybridNavigator`: both local and
  global costmaps must republish the committed morphology footprint before the
  next plan executes.
- The first implementation compared catalog coordinates with world-frame
  `published_footprint` coordinates and correctly failed closed. The recorded
  polygons showed that Nav2 had applied its 0.01 m padding and transformed the
  narrow rectangle: observed dimensions were approximately 1.64 by 0.40 m for
  the 1.62 by 0.38 m catalog footprint.
- Replaced the comparison with translation/rotation-invariant padded edge
  lengths. The next engineering run passed this synchronization barrier, proving
  that Nav2 consumed the narrow footprint, but RPP still reported an immediate
  projected collision on departure.
- A delayed command-line costmap capture produced no sample before shutdown, so
  it is not evidence. Add costmap diagnostics to the in-process mission observer
  rather than relying on timing an external subscriber.

Current inference: the post-transition failure is caused by local costmap
content or discretized projected footprint collision, not a stale compact
footprint. Preserve collision checking until the lethal cells and projected arc
are recorded and explained.

## In-process costmap evidence and benchmark clearance correction

- Added `controller_diagnostics` to immutable trial records. It captures the
  final local costmap class counts, up to 512 nearest high-cost cells, the robot
  pose, and RPP collision arc when published.
- Corrected the observer from unused `/local_costmap/costmap_raw`
  (`nav2_msgs/Costmap`, no publisher in this stack) to Nav2's lazily published
  `/local_costmap/costmap` (`nav_msgs/OccupancyGrid`) and enabled full costmap
  publication.
- The captured grid proved that the old 0.50 m doorway left only 0.05 m per side
  around the padded narrow footprint. A few degrees of residual yaw consumed
  that margin. An initial 0.60 m revision was off-center because it used an even
  cell count. The final generator uses a centered seven-cell (0.70 m) opening.
- Added a common 0.07-0.08 m operational safety envelope to the physical robot
  bounds. Compact is 0.72 m wide and therefore strictly rejected; narrow is
  0.54 m wide and retains 0.08 m per side. Updated planner fixture tests prove
  compact-only failure and hybrid reconfiguration.
- Tightened narrow RPP alignment to 0.01 rad and raised its angular acceleration
  limit to overcome low-speed contact stiction. This still needs a successful
  post-transition departure run.
- Fixed executor diagnostics to use the estimator's `visible` field. Before the
  fix, the formatter itself threw and could leave `TRANSITIONING`; afterward, a
  real pod-0 relocation timeout produced a structured failure and correctly
  entered `RECOVERY_REQUIRED`.
- Full-stack evidence in `results/debug/pilot_raw_centered_door2` verifies one
  reconfiguration attempt, terminal `unsafe_topology`, and no retry. Failure:
  pod 0, relocation waypoint 3/3, pose `(0.7257, 0.1234, -0.9516)`, visible
  fiducial/odometry estimate, detached latch observation.
- Removed redundant +/-90-degree terminal waypoint headings and increased the
  per-waypoint relocation timeout to 45 s with pod angular limit 0.9 rad/s. The
  next attempted run was an infrastructure failure because Nav2 lifecycle
  activation timed out; it yielded no mechanical evidence.
- Added controller lifecycle `ACTIVE` to runner readiness. Readiness timeouts are
  now labeled `infrastructure_failure`, and partial transitions increment the
  reconfiguration-failure count.

## Doorway traversal, drive diagnosis, and analysis precision

- Revised longitudinal approaches completed all pod relocations and committed
  the narrow topology in roughly 43 simulated seconds.
- `pilot_raw_collision_monitor` crossed the centered doorway with the raw-lidar
  collision monitor enabled, then failed progress beyond it.
- During nominal pure negative yaw, odometry and AMCL both show large translation
  and little rotation. Opposing pod commands were about ±0.02 m/s because of the
  ±0.10 m track. Residual attachment error and low yaw authority are the leading
  hypotheses.
- The next engineering artifact must retain every terminal dock pose and qualify
  straight/positive-yaw/negative-yaw motion after reconfiguration.
- Confirmatory randomization output now includes exact assignment counts,
  attainable exact p-value resolution, and Monte Carlo p-value standard error.

## Post-transition qualification and narrow yaw experiments

- Added an automatic post-transition motion gate before Nav2 can resume. It
  commands positive/negative yaw and forward/reverse motion, evaluates odometry
  in the starting body frame, serializes every stage into the navigation result,
  and inhibits assembled drive on failure.
- Added sensor-derived pod alignment snapshots on topology or execution-state
  changes. Final `READY` snapshots now retain all pod poses, yaw, covariance,
  visibility, sources, and sensing revision in immutable trial records.
- Tightened pre-latch convergence to 0.008 m and 0.025 rad; post-latch acceptance
  uses 0.012 m and the catalog connector limit of 0.05236 rad.
- Added a distinct `motion_qualification_failure` outcome and method-level
  qualification counts to analysis.
- Fixed batch cleanup to retain launch descendants and terminate reparented
  Gazebo processes. The prior orphan reproduced and was removed; subsequent
  runs left no `gz sim` process.
- `tight_docking_gate_raw` proved the gate detects the original failure and
  inhibits drive. `prelatch_alignment_raw` showed final yaw errors near 0.023 rad
  but almost no yaw authority at +/-0.025 m/s tangent speed.
- A cataloged narrow yaw-effort scale of 3.0 raised tangent speeds to +/-0.075
  m/s. `yaw_effort_scale_raw` produced correct +0.405/-0.429 rad yaw and clean
  straight/reverse motion, but a repeat with a different final pod-5 yaw sign
  failed. This establishes sensitivity to residual latch geometry.
- Rejected two approaches after live tests: sending pod-local angular velocity
  canceled or biased array yaw; an observed-geometry wrench allocator strongly
  coupled straight/reverse motion into yaw. Both were removed.
- Making staggered center pods passive immobilized the fixed-joint assembly
  because zero-commanded simulated pod wheels do not free-roll. That experiment
  was also removed.

Current mechanical blocker: make connector seating repeatable or develop a
plant-identified allocation robust to the actual fixed-joint/contact dynamics.
The automatic gate prevents these states from entering autonomous traversal.
All artifacts in this section are engineering-only debug evidence.

## Connector seating and merged-skeleton actuator diagnosis

- Extended attach commands with the catalog connector transform. The Gazebo
  topology plugin now seats the pod model relative to the core, waits one physics
  update, and then creates the detachable fixed joint.
- Added latch-triggered estimator reanchoring: pre-seat wheel/fiducial samples are
  discarded and pod odometry is reanchored at the observed connector target
  before post-latch validation.
- `seated_reanchored_raw` reached a final `READY` snapshot with all six fused
  poses essentially at their catalog coordinates (pod-5 yaw error about 0.002
  rad), demonstrating repeatable topology representation.
- Despite exact seating, opposing side commands still translated the assembly.
  The displacement closely equals the last pod's commanded speed integrated over
  the stage. This indicates the six stock Gazebo DiffDrive systems conflict after
  DART merges fixed-joint children into one skeleton; the last command effectively
  dominates instead of wheel forces combining.
- Isolated pod-local yaw commands also produced translation rather than yaw and
  were removed. The next plant fix is one assembly-level wheel-joint controller
  that writes all twelve uniquely named wheel joints coherently after docking,
  while detached pods retain their individual self-mobile controllers.

- A prototype world-level controller attempted to write all twelve wheel-joint
  velocity components coherently while the model DiffDrive systems were idle.
  The model systems overwrote those components regardless of SDF declaration
  order, leaving the assembly motionless. The prototype and bridge/config flags
  were removed. A valid ownership transfer requires replacing or explicitly
  disabling the per-model controller systems, not competing writes.

## Single wheel-controller and Bullet experiments

- Replaced the six model DiffDrive plugins experimentally with one world system
  that subscribed to all pod commands, wrote twelve wheel-joint velocities, and
  published integrated wheel odometry. With DART, scoped joint lookup and even
  pre-weld cached entity IDs still behaved as if only the final pod pair survived
  the weld; yaw commands translated at the final pod's speed.
- A world-level writer competing with model controllers was also ineffective:
  the model systems overwrote its commands. XML declaration order did not change
  that ownership.
- Bullet Featherstone was selected through the Physics system for one engineering
  run. The first detached pod could not converge to its relocation waypoint under
  the existing controller/tuning, so it did not provide assembled-motion evidence.
- These controller and engine prototypes were removed. The retained DART stack
  remains fail-closed at post-transition qualification. A future plant revision
  should avoid welding separately controlled Gazebo models, for example by using
  a single articulated robot model with explicit docking constraints and stable
  joint ownership.

## Prospective nuisance-grid implementation

- Added `morphology_study power-grid` to evaluate one predeclared absolute
  smallest effect over Cartesian combinations of control completion rate,
  layout-logit variance, and paired-noise fraction.
- Each grid cell uses a deterministic independent seed and retains its power
  estimate, Monte Carlo standard error, and assumptions.
- Output identifies the worst-case cell and gives an all-cells target-power
  decision, preventing selection of one favorable pilot nuisance estimate.
- Strengthened the decision after review: every cell now carries a 95% Wilson
  interval for simulated power, and the design passes only if every lower bound
  reaches the target. This prevents Monte Carlo noise around 0.80 from deciding
  the retained sample size.
- Updated the project status and critical path after the controller-ownership
  diagnosis. Confirmatory collection remains blocked at 0/432.
