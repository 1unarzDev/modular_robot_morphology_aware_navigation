# Project status and implementation phases — 2026-09-12

## Submission-level assessment

The project has a defensible research question, a working ROS/Gazebo platform
slice, four executable planning methods, generated held-out worlds, a resumable
mission runner, and a reproducible confirmatory analysis pipeline. It does not
yet have conference evidence because the platform has only one successful
round-trip qualification, the evaluator and perceived 3D context are incomplete,
and the frozen study contains zero terminal records.

The strongest paper contribution is the planner's decision boundary: it jointly
selects route and morphology while admitting a reconfiguration edge only when
the full module trajectory is collision/support/latch feasible and the docking
operation is observable at acceptable uncertainty. Daudelin, Pankert, Le,
Cheng, and Tu prevent broader first-system claims.

## Implemented status

| Workstream | Current state | Evidence |
|---|---|---|
| Self-mobile platform | Six differential-drive pods, physical DART detach/yaw/redock, compact and narrow assembled drive | One revised round trip passed; 20-run gate remains open |
| Topology safety | Observed topology revisions and `RECOVERY_REQUIRED`; drive inhibited after partial failure | Unit tests and live injected failure |
| Sensing | Wheel odometry plus idealized connector cameras; covariance, visibility, source and staleness gates | Unit tests and live topics; no physical perception claim |
| Hybrid planning | Weighted A* on `(x,y,heading,morphology)` plus explicit route-first baseline | Host golden tests |
| Transition feasibility | Sequential world-frame trajectories with compound pod body/wheel collision bounds and structured rejection reasons | Host tests distinguish planar, 3D, and sensing cases |
| Plan validity | Method/map/topology/sensing revisions; replan after reconfiguration or meaningful revision change | Host tests and nine-package ROS build |
| Statistics | Frozen paired design, immutable records, layout bootstrap, sign-flip tests, Holm adjustment, calibration, power simulation, deterministic artifacts | Host tests; 0/432 confirmatory records |
| Mission automation | Generated SDF/manifest per trial, isolated full-stack process, readiness gates including fresh lidar and odometry, simulated/wall deadlines, immutable resumable records | Debug engineering runs only; no valid pilot records |

Current automated verification: 73 host tests pass; all nine ROS packages build
and package smoke tests pass. The confirmatory design hash remains
`e9115d543af0969db7825398f7e2691361bdb536530d93e49db4528f21c51cf5`.

The current ROS sensing policy applies the latest discrete pod observability
signature to candidate transitions. Phase 2 must replace that temporary online
input with a location-dependent prediction from the perceived environment;
otherwise a future transition site would inherit observability measured at the
robot's current location.

A dedicated motion qualification run now verifies ROS REP-103 signs in the
physical array: commands of `+0.35` and `-0.35 rad/s` produced yaw changes of
`+0.881` and `-0.821 rad`, and a straight command produced `+0.762 m` with
negligible yaw. This rules out the plant sign convention as the cause of the
earlier MPPI divergence.

Both confirmatory morphologies now use regulated pure pursuit. In the latest
valid full-stack debug run, AMCL, odometry, and the controller tracked the
straight segment and reached the planned transition site near `(2.05, 1.85)`.
An earlier compact-to-narrow transition failed after roughly 80 s and left the
manager in `RECOVERY_REQUIRED`; the navigator incorrectly attempted the same
transition four times. Current edits block planning and execution unless the
observed state is `READY`, abort immediately after an unsafe partial transition,
add phase/pod/relative-pose/covariance/visibility/latch diagnostics, require
fresh scan and odometry at runner readiness, and classify transition and unsafe-
topology failures separately.

After rebuilding, two combined-constraints engineering reruns each completed one
compact-to-narrow transition and returned `READY`. Both then failed immediately
when regulated pure pursuit predicted a collision on the post-transition
traverse. The records contain one reconfiguration attempt; the remaining rapid
retries are controller retries. A later centered-door run produced a natural pod-0 relocation timeout and
directly qualified the unsafe-state branch: one transition attempt, structured
pod/pose/covariance/visibility/latch diagnostics, terminal
`RECOVERY_REQUIRED`, and no retry. The active mechanical blocker is repeatable
final-yaw convergence during pod relocation; revised arc waypoints await a valid
lifecycle-active rerun. Post-transition narrow departure also remains unproven.

## Phase 1 — platform qualification

Continue repeated runs after the first revised round trip passed in 70.118 s
forward and 84.081 s reverse. Add action feedback and pod-specific estimate/odometry/command/truth
traces for failures. Test straight, reverse, yaw, and lateral staging maneuvers
under randomized friction and initial-pose perturbations. Complete the declared
failure-injection matrix and verify every partial topology keeps assembled drive
inhibited.

Exit artifact: 20 consecutive round trips, a machine-readable qualification
summary, and a failure matrix. These runs stay outside pilot and confirmatory
data.

## Phase 2 — experimental environments and planner manipulation checks

The 12 parameterized layouts for each family and their SDF exporter are
implemented. Complete their online perception and manipulation checks:

1. Reconfiguration workspace: a short route whose apparent 2D clearance hides
   an invalid 3D pod trajectory, plus a longer valid staging route.
2. Docking observability: geometrically valid alternatives with different
   connector visibility and predicted covariance.
3. Combined constraints: workspace and observability favor different candidate
   sites so the full method must trade route cost against transition risk.

At least one quarter of layouts in each family should be neutral controls where
all methods should agree. Generate worlds from saved parameters, not hand-edited
SDF files. Add golden tests that assert each method's route signature,
transition site, and structured rejection reason. Supply actual perceived 3D
obstacles to the planner; an empty environment tuple does not test the 3D claim.

Add a saved occupancy map for each generated layout and localize against it from
lidar/odometry. Build the 3D transition context and observability field from
simulated sensor observations; scenario-manifest truth remains evaluator-only.

Exit artifact: frozen scenario manifests and a decision-separation report that
shows each ablation changes only its intended planner input.

## Phase 3 — unattended ROS/Gazebo mission runner

The first runner now restarts the full stack per trial, applies the four paired
seed streams, sends `NavigateHybrid` with the frozen method ID, enforces 300 s
of simulated time and a wall-clock watchdog, and writes one immutable terminal
record. Planner latency, expanded-state count, revisions, route signature, and
transition sites are carried by the navigation result. Complete process health,
collision/clearance, evaluator-only localization error, transition predictions,
edge-decision audits, recovery counts, and failure taxonomy.

Run trials sequentially until DDS and Gazebo isolation is demonstrated. Treat
only declared infrastructure failures as rerunnable and retain an audit entry
for the original attempt.

Exit artifact: a disjoint pilot that completes without manual intervention and
passes the same completeness/manifest checks as the confirmatory study.

## Phase 4 — pilot, effect threshold, and design decision

Use pilot data to estimate runtime, control completion, layout heterogeneity,
and within-pair dependence. Before examining a confirmatory outcome, record the
smallest operationally useful absolute completion gain and its rationale. Run
prospective hierarchical power at alpha 0.025 for both the 36-layout all-family
contrast and the 24-layout sensing-family contrast, including conservative
nuisance assumptions. Increase layouts if either primary contrast is
underpowered. Freeze container digest, source revision, maps, parameters,
scenario manifests, and analysis version after this decision.

Exit artifact: signed pilot decision record, archived power JSON with Monte
Carlo error, and either confirmation or replacement of the current 432-trial
design.

## Phase 5 — confirmatory run and paper package

Execute complete randomized layout/replicate blocks and inspect only
infrastructure health and record completeness during collection. Once the
design is complete, run the deterministic analysis once. Lead the results with
the two paired completion contrasts and their cluster intervals; then show
deadline-penalized time, work, failure causes, calibration, localization,
planning cost, and representative route decisions.

Exit artifact: immutable raw records, audit log, generated tables/SVG figures,
one-command reproduction instructions, and a paper whose limitations state
that cameras and latches are idealized simulation components.

## Critical path

The fastest credible sequence is: reverse-transition diagnosis, round-trip
qualification, scenario generator, environment-aware ROS transition validation,
unattended runner, disjoint pilot, power/design decision, confirmatory blocks.
Expanding to Ackermann, omni, crawler, articulated, or stacked morphologies
before this sequence finishes would add platform claims without strengthening
the stated navigation contribution.

## Current status update: doorway traversal and narrow-drive blocker

Revised longitudinal pod approaches completed all six relocations and committed
`narrow_tandem` in about 43 simulated seconds. The subsequent
`results/debug/pilot_raw_collision_monitor` run crossed the centered 0.70 m
doorway with the sensor-driven collision monitor active. This closes the former
immediate-departure blocker but remains engineering evidence.

The post-doorway failure is now localized. From roughly 100--210 simulated
seconds, Nav2 commanded essentially pure rotation (`linear_x=0`,
`angular_z≈-0.20 rad/s`), while odometry advanced about 2.1 m, moved laterally
about 0.54 m, and changed yaw by only about 0.03 rad. AMCL closely agrees with
odometry. Each side received opposing speeds of only about 0.02 m/s because the
narrow track is ±0.10 m. Residual dock yaw/contact asymmetry and insufficient
yaw authority are therefore the leading mechanisms; localization correction is
not the leading explanation.

The next platform phase is to preserve all six sensor-derived terminal dock
poses and relative velocities, then run straight and positive/negative yaw
qualification immediately after reconfiguration. Assembled navigation must not
resume unless yaw commands produce useful signed yaw with bounded translation.
Evaluate tighter pre-latch alignment first, then allocation from observed pod
orientations or a wider physically defensible track. Tune RPP only after the
plant passes this gate. Keep the downstream collision monitor enabled.

The analysis pipeline now reports exact sign-assignment counts and attainable
p-value resolution, plus Monte Carlo standard error for simulated randomization
p values. The confirmatory design hash remains
`e9115d543af0969db7825398f7e2691361bdb536530d93e49db4528f21c51cf5` and the
confirmatory evidence count remains **0/432**.
