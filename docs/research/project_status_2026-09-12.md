# Project status and implementation phases — 2026-09-12

## Submission-level assessment

The project has a defensible research question, a working ROS/Gazebo platform
slice, four executable planning methods, and a reproducible confirmatory
analysis pipeline. It does not yet have conference evidence because the reverse
transition is not qualified, held-out confirmatory worlds are not implemented,
and the frozen study contains zero terminal records.

The strongest paper contribution is the planner's decision boundary: it jointly
selects route and morphology while admitting a reconfiguration edge only when
the full module trajectory is collision/support/latch feasible and the docking
operation is observable at acceptable uncertainty. Daudelin, Pankert, Le,
Cheng, and Tu prevent broader first-system claims.

## Implemented status

| Workstream | Current state | Evidence |
|---|---|---|
| Self-mobile platform | Six differential-drive pods, physical DART detach/yaw/redock, compact and narrow assembled drive | Forward transition live; reverse remains intermittent |
| Topology safety | Observed topology revisions and `RECOVERY_REQUIRED`; drive inhibited after partial failure | Unit tests and live injected failure |
| Sensing | Wheel odometry plus idealized connector cameras; covariance, visibility, source and staleness gates | Unit tests and live topics; no physical perception claim |
| Hybrid planning | Weighted A* on `(x,y,heading,morphology)` plus explicit route-first baseline | Host golden tests |
| Transition feasibility | Sequential world-frame trajectories with compound pod body/wheel collision bounds and structured rejection reasons | Host tests distinguish planar, 3D, and sensing cases |
| Plan validity | Method/map/topology/sensing revisions; replan after reconfiguration or meaningful revision change | Host tests and nine-package ROS build |
| Statistics | Frozen paired design, immutable records, layout bootstrap, sign-flip tests, Holm adjustment, calibration, power simulation, deterministic artifacts | Host tests; 0/432 confirmatory records |

Current automated verification: 67 host tests pass; all nine ROS packages build
and package smoke tests pass. The confirmatory design hash remains
`e9115d543af0969db7825398f7e2691361bdb536530d93e49db4528f21c51cf5`.

The current ROS sensing policy applies the latest discrete pod observability
signature to candidate transitions. Phase 2 must replace that temporary online
input with a location-dependent prediction from the perceived environment;
otherwise a future transition site would inherit observability measured at the
robot's current location.

## Phase 1 — platform qualification

Re-run both transition directions after the narrow-track geometry and waypoint
changes. Add action feedback and pod-specific estimate/odometry/command/truth
traces for failures. Test straight, reverse, yaw, and lateral staging maneuvers
under randomized friction and initial-pose perturbations. Complete the declared
failure-injection matrix and verify every partial topology keeps assembled drive
inhibited.

Exit artifact: 20 consecutive round trips, a machine-readable qualification
summary, and a failure matrix. These runs stay outside pilot and confirmatory
data.

## Phase 2 — experimental environments and planner manipulation checks

Implement 12 parameterized layouts for each of three families:

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

Exit artifact: frozen scenario manifests and a decision-separation report that
shows each ablation changes only its intended planner input.

## Phase 3 — unattended ROS/Gazebo mission runner

Build a runner that restarts the full stack per trial, applies the four paired
seed streams, sends `NavigateHybrid` with the frozen method ID, enforces 300 s
of simulated time and a wall-clock watchdog, and writes one immutable terminal
record. Capture process health, topic freshness, collisions, topology state,
planning decisions, localization error, work, transition predictions, and
failure taxonomy. Keep simulator truth in evaluator-only nodes.

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
