# Conference evidence roadmap

Updated 2026-09-12. This document separates implemented engineering from
experimental evidence and defines the gates for a defensible submission.

## Paper claim

The candidate contribution is a hybrid navigation method that jointly chooses
route and morphology while conditioning transition edges on full-trajectory
reconfiguration feasibility, connector observability, localization uncertainty,
and calibrated time/energy/failure costs. The study tests whether those added
constraints improve mission completion relative to sequential, geometry-only,
and feasibility-only planners under common mechanics and sensing.

The paper must not claim the first morphology-aware navigation stack, the first
ROS autonomous reconfiguration system, or novelty from searching pose and shape
alone. Daudelin et al. (2018), Le et al. (2018), Cheng et al. (2020), Pankert et
al. (2022), and Tu et al. (online 2025) bound those claims. Upcoming 2026
workshop talks are discovery leads rather than completed publications.

## Current evidence

| Area | Status | Evidence level |
|---|---|---|
| Hybrid state search and morphology footprints | Implemented and host-tested | Engineering |
| Confirmatory mechanics | Restricted to self-mobile differential pods in `compact_diff` and `narrow_tandem`; one revised DART round trip passed (70.118 s forward, 84.081 s reverse) | Engineering qualification; 20-run gate open |
| Observed topology and transition safety | Partial failures enter `RECOVERY_REQUIRED`; drive stays inhibited until reconciliation | Unit and one live failure check |
| Sensor-derived relative pod pose | Wheel odometry plus visibility-limited logical cameras; executor no longer consumes module ground truth | Unit and live topic check |
| 3D transition validation | Catalog paths expand into sequential world-frame compound body/wheel trajectories; collision, support, visibility, covariance, velocity, reach, and latch checks feed planner edges | Planner-wired host tests; live requalification pending |
| Planner ablations | Route-first sequential adaptation and three coupled methods have explicit shared-stack implementations and structured transition decisions | Golden host tests; scenario and ROS mission validation pending |
| Plan consistency | Plans carry method, map, topology, and meaningful sensing-signature revisions; navigator replans after a transition or revision change | Host/ROS build verification; live race/fault checks pending |
| Nav2 footprint interface | Installed Jazzy graph uses `Polygon` input and `PolygonStamped` output | Live graph check |
| Confirmatory statistics | Frozen-design model, immutable records, cluster bootstrap, paired randomization, Holm correction, failure taxonomy, calibration, power simulation, tables, and SVG figures | Tested analysis infrastructure |
| Held-out worlds and runner | Parameterized SDF worlds/manifests and a resumable full-stack runner; latest smoke reaches readiness but online SLAM has not observed the distant doorway | Debug engineering only |
| Main scientific claim | No balanced mission experiment completed | No result yet |

Logical cameras are idealized onboard observations. They support an uncertainty
and observability study in simulation; they do not validate fiducial detection
or physical latch capture. Gazebo joint attachment is likewise an execution
surrogate rather than proof of connector mechanics.

## Implementation and evidence gates

### Gate 1 — mechanically credible round trips

- Validate straight motion, in-place yaw, and reverse/diagonal approaches for a
  detached pod using both wheel odometry and evaluation-only world pose.
- Complete `compact_to_narrow` and `narrow_to_compact` repeatedly.
- Inject detach, relocation, latch, stale-feedback, manager-commit, and
  cancellation failures; confirm drive inhibition and observed partial topology.
- Reconcile every recoverable known topology to `READY`.

Exit criterion: at least 20 consecutive round trips over randomized friction
and pose perturbations, with no unsafe drive re-enable and a documented failure
matrix. This is a platform qualification run, excluded from confirmatory data.

### Gate 2 — planner variants share one execution stack

- Exercise the planner-wired 3D validator in ROS missions and bind perceived
  overhead geometry to its environment boxes.
- Verify all four implemented methods over the same graph, controller,
  estimator, retry budget, and map inputs.
- Log why every transition edge is accepted or rejected, including 3D collision,
  support, visibility, covariance, connector reach, and latch feasibility.
- Add neutral layouts where feasibility and sensing terms should make no
  difference, preventing a benchmark composed only of favorable examples.

Exit criterion: hand-inspected golden cases demonstrate the intended decision
separation, and removing one constraint changes only its planned ablation.

### Gate 3 — end-to-end sensor-based missions

- Replace every autonomy dependency on ground-truth module and robot pose with
  SLAM, odometry, IMU, lidar/RGB-D, and connector observations. Retain ground
  truth only in the evaluator.
- Localize from sensors against a generated prior occupancy map for the fixed-goal
  confirmatory task. Keep map truth common to all methods and disclose it as a
  prior. Treat online exploration as a separate experiment if pursued.
- Execute doorway, poor-observability docking, insufficient-workspace, combined,
  and neutral missions with collisions, topology, covariance, work, and timing
  recorded from the simulator.
- Make the batch runner restart ROS/Gazebo per trial, enforce the simulated-time
  deadline, classify infrastructure failures separately, and resume from
  immutable terminal records.

Exit criterion: a disjoint pilot completes without manual intervention and
every failure maps to the declared taxonomy.

### Gate 4 — freeze and run the confirmatory study

- Choose the smallest effect of interest before viewing confirmatory outcomes.
- Use pilot-only estimates in prospective power simulations for the 36-layout
  and 24-layout primary scopes; adjust the layout count if needed.
- Freeze design, container image/revision, source commit, configuration hash,
  maps, estimator parameters, and analysis version.
- Run randomized paired blocks and monitor only infrastructure health and record
  completeness. Do not inspect method outcomes mid-run.
- Analyze once all expected records pass the integrity checks.

Exit criterion: a complete immutable dataset, audit log, derived tables and
figures, and a one-command reproduction path from records to manuscript values.

### Gate 5 — submission package

- Report absolute effects and cluster intervals even when p values are not
  significant; include all robot failures and deadline penalties.
- Show completion, time/work tradeoffs, failure taxonomy, transition-risk
  calibration, localization error, planner latency, and representative route
  decisions.
- Release frozen designs, scenario generator, container manifest, raw records,
  evaluator, analysis artifacts, and negative results.
- Phrase physical relevance around simulated self-mobile modules and explicitly
  state the idealized sensing and latching limits.

The strongest conference narrative is the decision-level ablation: geometry
coupling establishes shape-aware planning, 3D feasibility prevents executable-
looking but invalid transitions, and sensing-aware feasibility shifts docking
to observable workspaces. The mission experiment then measures whether those
decisions improve completion under identical execution conditions.

## Immediate execution order

1. Rebuild the ROS workspace and rerun the preserved combined-constraints case.
   Confirm that one failed transition produces one attempt and an immediate
   unsafe-topology terminal record.
2. Use executor diagnostics to repair pod relocation or docking; then pass 20
   consecutive randomized compact-to-narrow-to-compact round trips and the
   declared injected-failure matrix.
3. Add evaluator-only synchronized truth, full-geometry clearance/collision,
   transition prediction/outcome, process-health, and recovery telemetry. No
   pilot starts until every declared analysis field has a measured source.
4. Bind sensor-derived, location-dependent 3D obstacles and connector
   observability to planner edges. Produce golden decision-separation cases and
   neutral controls for all four methods.
5. Run a disjoint unattended pilot, set the operational smallest effect of
   interest, and archive prospective power and precision results. Re-freeze the
   layout count if either contrast lacks power.
6. Freeze software and configuration artifacts, execute complete paired blocks,
   and release the immutable 432-record dataset only if the pilot retains the
   current design.
