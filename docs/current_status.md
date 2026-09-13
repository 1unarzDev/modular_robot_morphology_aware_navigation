# Current project status

Updated 2026-09-12. This is the authoritative handoff for implementation and
evidence state.

## Research claim

The proposed method jointly selects route, morphology, and reconfiguration site
while conditioning transition edges on full 3D module-trajectory feasibility,
connector observability, localization uncertainty, and time/energy/risk costs.
The study compares it with sequential route-first, geometry-only, and
feasibility-only alternatives under identical mechanics, sensing, maps, and
disturbances.

Prior work already establishes autonomous modular reconfiguration, shape-aware
planning, and simultaneous navigation/reconfiguration. The bounded contribution
and citations are maintained in the literature review.

## Evidence ledger

| Area | Current state | Evidence and limitation |
|---|---|---|
| Hybrid planner | Weighted search over `(x, y, heading, morphology)` with four executable methods | Host golden tests cover route and transition decisions |
| Reconfiguration validation | Sequential compound pod/body/wheel trajectories, stationary-core collision, support/latch checks, sensing gates, and structured rejection reasons | 3D checks are geometric simulation validation, not physical certification |
| Self-mobile platform | Six independently mobile differential pods; one controller owns all wheel joints; bounded effort actuation and vertical wheel suspension | Detached and compact motion gates pass |
| Reconfiguration | Sensor-driven detach, relocate, align, latch, topology verification, and fail-closed recovery | One compact-to-narrow run reaches `READY`; reverse transition and repetition gates remain open |
| Navigation | ROS 2 Jazzy, Nav2, lidar/odometry localization, morphology-specific footprints/controllers, collision monitor | Doorway traversal exists only as engineering evidence |
| Sensing | Wheel odometry and visibility/staleness/covariance-gated connector observations | Connector camera and latch remain idealized; location-dependent predicted observability is incomplete |
| Experiment runner | Deterministic paired seeds, randomized method order, generated SDF/manifests, isolated process launch, immutable terminal records, resumability | Debug runs only; no accepted pilot records |
| Statistics | Layout-balanced estimands, stratified bootstrap, sign-flip tests, Holm correction, power grids with Wilson bounds, and hierarchical logistic sensitivity analysis | Pipeline-tested; no confirmatory outcomes |

The prospective design contains 432 trials and has hash
`e9115d543af0969db7825398f7e2691361bdb536530d93e49db4528f21c51cf5`.
Confirmatory evidence is **0/432**.

## Supported scope

Only `compact_diff` and `narrow_tandem` are confirmatory morphologies. Both use
ground-contact differential pods. `ackermann`, `omni_wide`, `long_crawler`,
articulated, and stacked concepts are catalog placeholders and must remain out
of results and physical claims.

The supported tracks place pod centers at ±0.20 m to clear the 0.20 m-wide core.
The narrow safety footprint is 0.66 m wide, compared with compact's 0.72 m,
preserving the planned 0.70 m doorway distinction.

## Active platform blocker

Wheel suspension resolved compact-assembly contact loading. With native Gazebo
`JointVelocity` feedback, detached signed yaw is +1.0536/-1.0540 rad with
+0.7054/-0.7050 m forward/reverse travel; compact signed yaw is
+0.6496/-0.6407 rad with at most 0.00274 m translation and 0.7455 m straight
travel. Both prerequisite runtime gates pass.

The native-feedback replay exposed and repaired callback-order-dependent sensing
revision ownership and a docking stability check that conflated fresh samples
with planning-relevant sensing changes. With the pure-turn floor restored to
0.30 rad/s, `results/debug/native_velocity_transition_latest22_raw` completed
all six relocations, committed topology revision 13, and returned to `READY`.
Its post-transition gate still failed safely: both yaw stages produced exactly
zero core yaw, while forward/reverse travel was +0.08926/-0.08926 m.
Evaluator-only wheel feedback has the correct signs, but pods translate
independently under yaw allocation while the core remains stationary. The
remaining blocker was traced to two-wheel pod pitch and excessive lateral wheel
scrub. Low-friction fore/aft caster contacts now stabilize each self-mobile pod,
and the fixed-wheel model declares lower secondary friction (`mu2=0.08`) while
retaining longitudinal traction. Suspension travel and velocity are now included
in evaluator-only drive diagnostics.

Two identical-seed engineering missions completed with two transition attempts,
`READY` execution state, and no recovery fault. Their first post-transition yaw
gates measured +0.4519/-0.5273 rad and +0.4514/-0.5165 rad; their second gates
measured +0.2749/-0.2754 rad and +0.2828/-0.2812 rad. Both missions completed in
159.884 and 162.052 simulated seconds. These are repeat engineering checks, not
pilot or confirmatory evidence. The planned second transition is
`narrow_to_compact`. The apparent missing final topology event was an evidence
ledger bug: connector revision does not change at morphology commit. Terminal
records now contain `final_morphology`, topology history records morphology-only
commit events, and the navigator waits for target morphology, `READY`, and an
advanced connector revision before accepting transition success.

`results/debug/ack_roundtrip_latest34_raw` verifies the corrected contract. It
completed in 161.098 simulated seconds with two successful transitions, final
`compact_diff`, `READY`, no unrecovered fault, and passing signed-motion gates
after both transitions. Reverse transition is now qualified for this engineering
seed; randomized repetition remains open.

Applied plant randomization is now wired through the world manifest, Gazebo SDF,
and AMCL initialization. `friction_seed` realizes ground friction in [0.70,
1.10], an x/y spawn offset in +/-0.015 m, and a yaw offset in +/-0.035 rad. The
paired methods receive the same realization. In
`results/debug/disturbed_roundtrip_latest36_raw`, ground friction 0.8186 and pose
offset (-0.00435, +0.00039, +0.02212 rad) produced a completed 161.108 s mission,
two successful transitions, final `compact_diff`/`READY`, and two passing
signed-motion gates. This is one engineering realization and is not reliability,
pilot, or confirmatory evidence. Readiness timeouts now retain every predicate in
their terminal message for infrastructure diagnosis.

Gate 0 now has a frozen 20-run engineering round-trip design at
`config/engineering_roundtrip_design.json` with hash
`9c74a26749d466ba382caa6b4d227a033139ef24b8600a3d10ef90608a088b5e`.
The batch auditor requires every immutable terminal record, two successful
transitions, the compact-narrow-compact commit sequence, final `READY`, both
signed-motion gates, all-pod rigidity, and 20 unique realized disturbances.
Per-pod evaluator diagnostics use independent topics and a separate high-rate
3D pose ledger, preventing one pod from being hidden by shared bridge-queue
starvation. Rigidity is the maximum change in each pod's SE(3) transform relative
to pod 0 during each post-transition motion window; the engineering bounds are
0.015 m and 0.03 rad with at least three synchronized samples per pod.

`results/debug/rigidity_roundtrip_latest43_raw` is the first successful harness
validation. It completed with two transitions, final `compact_diff`/`READY`, and
both motion and rigidity gates passing. Every pod had 61 synchronized samples in
each window. This is 1/20 engineering runs in an isolated debug directory; it is
not part of the eventual immutable 20-record qualification directory and does
not establish reliability.

The first independent campaign, `results/qualification/roundtrip_20_raw`, is
retained as a failed qualification: 19/20 missions completed and replicate 15
ended in `infrastructure_failure` before navigation. Nav2 had activated and was
publishing its local costmap, but the observer missed the volatile lifecycle
event and its lifecycle service query timed out. The other 19 completed runs had
155.090--158.194 s simulated durations; across 190 pod/window measurements the
maximum relative-transform drift was 1.81e-6 m and 2.08e-6 rad with 60--61
synchronized samples. All 20 disturbances were unique, spanning friction
0.7169--1.0854 and the declared pose ranges. These figures diagnose the failed
engineering campaign and are not pilot or confirmatory results.

Readiness now also accepts a fresh local-costmap publication as authoritative
evidence that the controller lifecycle node is active. An exact-seed replay of
replicate 15 in `results/debug/roundtrip_r15_repro_raw` then completed both
transitions and passed both motion and rigidity gates. A new independent 20-run
campaign is still required; the replay does not replace the preserved failure.

## Resume here

Execute the frozen 20-run engineering design in a new immutable raw directory.
Each run must contain two successful planned
transitions, commit `narrow_tandem` and return to `compact_diff`, finish `READY`,
pass both signed-motion gates, retain unique realized disturbances, and show no
unrecovered fault. Quantify realized plant variation across those runs rather
than inferring it from seed metadata. After that, execute the declared
fault-injection matrix.

After mechanics pass, implement location-dependent perceived 3D obstacles and
observability, complete evaluator outcome/provenance streams, run a disjoint
pilot, freeze the operational effect threshold and conservative power grid, and
only then freeze and execute the confirmatory schedule.

## Last verified checkpoint

- Platform commit: `d77c2a7` (`Qualify bounded self-mobile pod mechanics`).
- Native-feedback and sensing-protocol fixes are currently uncommitted.
- `python3 -m pytest -q`: 90 passed in 161.95 s.
- `colcon build --symlink-install`: all nine packages passed.
- `colcon test`: all five packages containing smoke tests passed; the remaining
  four contain no package-level tests.
- Documentation links/stale references and `git diff --check` passed.
- No simulator or bridge processes remained after verification.
