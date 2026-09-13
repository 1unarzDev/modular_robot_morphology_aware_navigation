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

The second campaign, `results/qualification/roundtrip_20_retry1_raw`, retained
18 completed missions and two intermittent pre-mission readiness failures. This
does not block the study: the runner now permits at most two retries only for a
live launch that fails before mission execution. Every attempt has a distinct
log, an fsync'd JSONL ledger, and embedded terminal-record provenance. Crashes,
robot failures, navigation failures, and missions that start executing are never
retried. Infrastructure-attempt rates will be reported separately from mission
outcomes.

Fail-closed recovery now has a declared seven-case executor fault matrix
(`engineering_fault_matrix`). Each case injects one failure into the first
`compact_to_narrow` transition through the launch `failure_injection` argument.
After the terminal record is captured, the evaluator commands 0.10 m/s body
motion for 2 s and then requests topology reconciliation. The debug execution
`results/debug/gate0_smoke_fault` (design hash
`f1e172305fe3fb29f98a7fb54468fea16885090093a04111e1aaf0e73cfcf29e`,
`reconfiguration_workspace-00`, `sensing_feasibility_coupled`, master seed 1)
passed all seven cases with one launch attempt each:

| Case | Observed topology | Reconciliation |
|---|---|---|
| detach, stale observation, cancellation | complete compact | `READY` as `compact_diff` |
| relocation, latch (`pod_0`) | `pod_0` detached | refused; `RECOVERY_REQUIRED` |
| partial topology (`pod_1` latch) | `pod_0`/`pod_2` narrow, `pod_1` free | refused; `RECOVERY_REQUIRED` |
| manager commit | complete narrow | `READY` as `narrow_tandem` |

Every case ended `unsafe_topology` with zero pod commands and zero measured core
translation or yaw during the drive stimulus. Injected cancellations now abort
the action goal because no client cancel request exists. This is one layout and
one seed per case; it is engineering evidence, not reliability evidence.

Bounded retry provenance is validated: a frozen-design round trip in
`results/debug/retry_provenance_20260913_220228` completed on its first launch
with a matching attempt ledger and embedded record provenance. Terminal records
now carry `phase_timing` (setup, stack readiness, mission wall time and
real-time factor, teardown). On the 8-core, GPU-less development container a
round trip spends about 0.02 s in setup, 14--17 s reaching readiness, and under
1 s in teardown; mission execution dominates. Profiling found
`hybrid_navigator` and `reconfiguration_executor` each consuming about 87% of a
core under rclpy's four-thread executor. An isolated benchmark showed that
executor costs about 3x more CPU per 50 Hz coroutine polling wake than the
single-threaded executor (43% vs 15% of a core); reusing timers or using two
threads did not help. Both nodes now use the single-threaded executor already
used by every other node, and all of their blocking waits are bounded. Neither
node remains among the top CPU consumers, host load fell from 9--13.5 to 7--9,
and Gazebo holds about 1.0 real-time factor. Wall-clock variance between
identical runs remains large (one earlier identical round trip took 254 s), so
these are engineering throughput measurements, not benchmarks. The RGB-D camera
still renders in software every trial without a consumer; it is retained for
planned perceived 3D obstacles.

## Resume here

Repeat the fault matrix across layouts before the pilot, then proceed to the
disjoint pilot without spending more submission time on repeated 20-run startup
certification. If throughput is still limiting, measure trial wall time with
the RGB-D camera disabled before deciding whether perceived-obstacle work needs
it at 15 Hz.

After mechanics pass, implement location-dependent perceived 3D obstacles and
observability, complete evaluator outcome/provenance streams, run a disjoint
pilot, freeze the operational effect threshold and conservative power grid, and
only then freeze and execute the confirmatory schedule.

## Last verified checkpoint

- Platform commit: `c41a0db` (`Repair fault-matrix rebase onto retry provenance`).
- Two independent engineering campaigns retain 37/40 completed missions; all
  three failures occurred before mission execution.
- Fault matrix: 7/7 cases passed in one engineering execution; the refactored
  generator reproduces its frozen design hash and the records re-summarize as
  passing.
- Single-threaded executor regression: a frozen-design round trip completed with
  both motion and rigidity gates passing at mission real-time factor 0.95, and
  the identical seven-case fault matrix passed again with the same
  reconciliation outcomes in 405.8 s of summed trial wall time (519.4 s before
  the change).
- `python3 -m pytest -q`: 111 passed in 148.61 s.
- `colcon build --symlink-install`: all nine packages passed.
- `colcon test`: all five packages containing smoke tests passed; the remaining
  four contain no package-level tests.
- Documentation links/stale references and `git diff --check` passed.
- No simulator or bridge processes remained after verification.
