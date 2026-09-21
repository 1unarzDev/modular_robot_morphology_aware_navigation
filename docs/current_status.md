# Current project status

Updated 2026-09-20. This is the authoritative handoff for implementation and
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
| Reconfiguration validation | Sequential compound pod/body/wheel trajectories, stationary-core collision, support/latch checks, sensing gates, the declared post-transition verification maneuver, and structured rejection reasons | 3D checks are geometric simulation validation, not physical certification; the maneuver sweep is the commanded nominal motion, with execution tolerance unmodelled |
| Self-mobile platform | Six independently mobile differential pods; one controller owns all wheel joints; bounded effort actuation and vertical wheel suspension | Detached and compact motion gates pass |
| Reconfiguration | Sensor-driven detach, relocate, align, latch, topology verification, and fail-closed recovery | One compact-to-narrow run reaches `READY`; reverse transition and repetition gates remain open |
| Navigation | ROS 2 Jazzy, Nav2, lidar/odometry localization, morphology-specific footprints/controllers, collision monitor | Doorway traversal exists only as engineering evidence |
| Sensing | Wheel odometry and visibility/staleness/covariance-gated connector observations | Connector camera and latch remain idealized; location-dependent predicted observability is incomplete |
| Experiment runner | Deterministic paired seeds, randomized method order, generated SDF/manifests, isolated process launch, immutable terminal records, resumability | Debug runs only; no accepted pilot records |
| Statistics | Layout-balanced estimands, stratified bootstrap, sign-flip tests, Holm correction, power grids with Wilson bounds, and hierarchical logistic sensitivity analysis | Pipeline-tested; no confirmatory outcomes |

The prospective design contains 432 trials and has hash
`e9115d543af0969db7825398f7e2691361bdb536530d93e49db4528f21c51cf5`.
Confirmatory evidence is **0/432**.

Read the Gate 0 rows above with the retention finding below: the records
behind them no longer exist, so those entries are unverifiable prose rather
than auditable evidence.

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

Gate 1 measurement work has started. A survey found that analysis already
consumes collision, clearance, localization error, transition calibration,
edge-decision, and recovery-action fields, but the runner left all six at
their defaults. Terminal records now populate `localization_error_m`: each 2 Hz
map-frame estimate is compared with the nearest same-tick evaluator core pose
within 0.25 s (generated worlds, maps, spawn poses, and AMCL initial poses
share one frame). On the executor-regression round trip every one of 326
estimates had synchronized truth, with 4.95 cm RMSE and 7.97 cm maximum error.
`evaluator_metrics.telemetry_audit` rejects dropped diagnostics, missing truth,
sparse localization, unsynchronized samples, truth that does not span the
estimate window, and recorded errors that disagree with recomputation. Its
first use exposed a record-integrity bug: fault-trial records passed live
observer lists, so localization, odometry, command, topology, and execution
histories absorbed samples from the post-terminal recovery probe. Terminal
observations now snapshot every stream. A static test also rejects any
reference to evaluator topics, `/module_pose` (bridged Gazebo truth without an
`/evaluator` prefix), or evaluator pose fields in autonomy packages. All three
gaps this paragraph previously listed are closed, and the merge of the workshop
branch is what closed them; the claim that they were open was stale. Collision
and 3D clearance are computed by `evaluator_metrics.clearance_metrics` from
evaluator-only module poses against occupancy-derived boxes and the declared
transition obstacles. `NavigateHybrid` returns a pre-action
`predicted_failure_probability` beside the observed transition outcome, and
`transition_calibration_pairs` consumes them. `recovery_actions` is populated.

Three limits on that remain, and they are limits of definition rather than of
wiring. Contact is geometric, not physical: it is clearance at or below zero,
debounced at 0.5 s per module, computed from poses rather than read from a
simulator contact sensor, so it cannot see forces or contacts with undeclared
geometry. Clearance is only evaluated within 1.0 m of a known obstacle, so
`evaluated_samples` is the denominator and not the sample count. And
`recovery_actions` counts navigator replans after an execution failure only --
Nav2 recovery behaviors, executor `RECOVERY_REQUIRED` reconciliations, and
docking retries are not included in it, so it must not be reported as a total
recovery count.

## Transition feasibility now covers the motion that follows the commit

The workshop diagnostic exposed a gap the transition model did not cover: at a
site the feasibility check accepted, the executor committed `narrow_tandem` and
the automatic post-transition motion check then yawed the 1.6 m body in place
and drove pod 0 into the obstruction. Relocation was modelled; the verification
maneuver performed at the same site was not.

The maneuver is now declared once, as `post_transition_verification` in
`src/modular_robot_description/config/morphologies.yaml`, and consumed by both
sides: `morphology_planner` sweeps it when deciding whether a site can host a
transformation, and `modular_robot_bringup.qualification.VERIFICATION_SEQUENCE`
executes it. `tests/test_post_transition_qualification.py` binds the two, so
the planner cannot accept a site against a maneuver the robot does not perform.

The check sweeps the target morphology rigidly -- core and every pod -- through
the commanded twists from the committed pose, and rejects the edge with
`<module>:post_transition_collision`. It is cached per (morphology, heading)
and translated per site, as the relocation sweep already is. It applies to
`feasibility_coupled` and `sensing_feasibility_coupled` only; `geometry_coupled`
is unchanged, so the ablation contrast is preserved.

Scope: the sweep covers declared 3D transition obstacles, which the 2D
occupancy map cannot represent. Walls remain covered by the planar clearance
disk, which is valid only while `swept_radius` exceeds the maneuver's reach
(0.92 m for `compact_to_narrow` against a 1.0 m disk); a test asserts this for
every transition. `margin_m` is declared and set to 0.0, so the sweep is the
commanded nominal motion and execution tolerance is not yet modelled.

That gap is measurable and is not currently conservative. The maneuver commands
0.375 rad of yaw per stage, but the recorded engineering post-transition gates
measured up to 0.5273 rad, 41% more. At the farthest swept corner (0.9534 m)
that is 0.145 m of arc the planner does not sweep, so in principle the check
can accept a site the executed maneuver then strikes. Re-sweeping the Blocked-A
scenario at the observed 0.5273 rad shows the current decisions are unaffected:
the rejected site at 1.85 m collides under both the commanded and the observed
angle (51 and 54 swept boxes), and the chosen site at 1.55 m collides under
neither. Derive the tolerance from the Gate 0 record set rather than from these
two runs, and note that `margin_m` inflates boxes linearly while the error is
angular and therefore grows with radius.

A related gap is checked and currently latent. `_state_is_free` and
`_traversal_is_free` consult only the occupancy grid, so declared 3D
transition obstacles are invisible to ordinary traversal edges for every
method -- including the `raised_transition_shelf`, which spans z 0.10--0.18 m
and therefore does intersect the pod and core envelopes. Sampling the
assembled 3D body along the planned routes of `reconfiguration_workspace`
(1, 8) and `combined_constraints` (1, 8) found no intersection, so no current
layout routes through a 3D obstacle. This was a coarse check, not a proof.
Closing the gap properly would mean checking 3D obstacles on traversal edges,
which changes what `geometry_coupled` is defined to ignore; that is a study
design decision and must not be made silently.

The nine workshop missions have been re-run end to end through Gazebo at
commit `92cae6f`, against the same frozen design hash
`9c608ab3e9d128af2471452f76bd6e1c24ab863b95b4825f45100217005376c2` the
published run used, so the two are directly comparable. All nine records were
accepted. On the frozen workshop scenarios the planner change moves exactly one
decision, and it is the one the diagnostic showed to be physically wrong:

| Variant | Method | Site before | Site after |
|---|---|---|---|
| Blocked-A | `geometry_coupled` | 2.15 m | 2.15 m |
| Blocked-A | `feasibility_coupled` | 1.85 m | **1.55 m** |
| Blocked-B | `feasibility_coupled` | 1.85 m | 1.85 m |
| Neutral | all three | 2.15 m | 2.15 m |

The Blocked-A rejection at 1.85 m is attributed to `pod_0`, which is the pod
observed in contact during the failed verification maneuver. Neutral produces
no rejections, so the added term still creates no artificial difference in the
unobstructed control.

Execution confirms the prediction. Both Blocked-A missions that previously
staged at 1.85 m, transformed, and then failed their verification yaw now stage
at 1.55 m and complete:

| Cell | Published | Re-run |
|---|---|---|
| Blocked-A `route_first_adaptation` | 1.85 m, `motion_qualification_failure`, 1 collision | 1.55 m, completed in 118 s, 2.9 cm clearance, 0 collisions |
| Blocked-A `feasibility_coupled` | 1.85 m, `motion_qualification_failure`, 1 collision | 1.55 m, completed in 164 s, 2.6 cm clearance, 0 collisions |
| Blocked-A `geometry_coupled` | 2.15 m, `unsafe_topology`, 8 collisions | 2.15 m, `unsafe_topology`, 7 collisions |
| Blocked-B, all three | unchanged | unchanged |

The geometry-only baseline still fails at 2.15 m in both blocked variants, so
the ablation contrast survives the change rather than being erased by it.

One cell moved the other way and is not attributable to the planner. Neutral
`geometry_coupled` completed in the published run and ended
`motion_qualification_failure` in the re-run, with zero collisions and 18.3 cm
minimum clearance: it struck nothing. Its reverse stage travelled -0.0788 m
against the -0.08 m floor, missing by 1.2 mm, and its forward stage reached
only 0.0813 m where every other mission reached about 0.113 m. Neutral has no
transition obstacles and `geometry_coupled` never evaluates the verification
sweep, so its planner decision is provably identical. Three identical-seed
repeats in `results/debug/neutral_geometry_repro_{1,2,3}_raw` all completed,
with reverse travel -0.1129, -0.1129, and -0.1129 m.

That intermittency is now diagnosed, and it was neither the gate threshold, the
actuator allocation, nor the contact model. `_command_for` bounded each
commanded maneuver stage with `time.monotonic()` while the robot moved in
simulated time, so a stage delivered `duration * real_time_factor` simulated
seconds of motion and travel scaled with host load. The recorded simulated
windows are conclusive: all six passing qualifications ran 6.00--6.02 s against
the 6.0 s nominal sequence, and the failing one ran 5.00 s. Within that run the
first stage was unaffected (positive yaw +0.4413 rad against a +0.4472--0.5028
norm) and the remaining three were uniformly short at 67--72% of norm --
negative yaw -0.3459 rad, forward +0.0813 m, reverse -0.0788 m -- which is the
signature of a sustained real-time-factor drop, not of a mechanical fault. No
single mechanical cause scales yaw, forward, and reverse by one common factor
while lateral coupling stays under 1.1 mm and minimum clearance stays at
18.3 cm.

The window is now measured on the simulated clock, the same time base the
recorded odometry poses carry. `CommandWindow` in
`modular_robot_bringup.qualification` owns the rule, and the wall clock is
retained only as a stall backstop: a simulated clock that stops advancing ends
the stage with `motion_command_window_unavailable` after
`motion_command_stall_grace_s` rather than blocking until the trial watchdog.
Each stage now records `commanded_duration_s` and `simulated_duration_s`, and
`TrialRecord.validate` rejects any record whose stage ran less simulated time
than it commanded, so a truncated verdict cannot enter a record set again.
Records predating those fields are unchecked and remain valid. The standalone
`modular_robot_benchmarks.motion_qualification` engineering tool already
advanced its stages on `node.sim_time` behind a wall watchdog; the navigator had
diverged from that pattern and now matches it.

The same defect was latent one node downstream and is fixed with it.
`assembly_drive_adapter` compared command staleness against `time.monotonic()`
while its publisher and its commander are both paced by simulated time, so with
a 0.3 s timeout and a 0.05 s republish period it zeroes a live command once the
real-time factor drops below about 0.17. That was unreachable while the window
itself ended early under load, and reachable afterwards. Staleness is now
measured on the node's ROS clock, and an unset command time is explicitly
stale rather than arithmetically fresh at simulated time zero.

The fix is confirmed at runtime, and the confirming runs are stronger evidence
than staged ones would have been. Two replays of the same frozen design and
seed at commit `c68281a` landed on different host loads without being asked to.
`results/debug/simclock_neutral_baseline_raw` realized a 0.5377 mission
real-time factor -- far below the 0.94--0.98 of every earlier run, and below
the factor that broke the published cell -- and
`results/debug/simclock_neutral_repeat_raw` realized 0.8806. Both completed with
zero collisions and both gates passing, at travel indistinguishable from the
unloaded norm:

| Run | RTF | forward (1st/2nd gate) | reverse (1st/2nd gate) |
|---|---|---|---|
| `simclock_neutral_baseline` | 0.5377 | +0.11341 / +0.11721 m | -0.11622 / -0.11426 m |
| `simclock_neutral_repeat` | 0.8806 | +0.11275 / +0.11288 m | -0.11289 / -0.11287 m |
| Unloaded norm (3 repros) | 0.97--0.98 | about +0.113 m | about -0.1129 m |

Commanded travel is therefore invariant across a 0.54--0.98 real-time-factor
span, where before it was proportional to it. That also closes the diagnosis
quantitatively. With a 0.113 m travel norm and a 0.08 m floor, the old
wall-clock window had to breach the floor whenever the stage-local real-time
factor fell below 0.08/0.113 = 0.708. The failing run's three truncated stages
came in at 67--72% of norm, straddling that ratio exactly, and its first stage
-- which ran before the slowdown -- was unaffected. Every stage received at least its
commanded window (1.000--1.034 s against 1.00 s, 1.500--1.536 s against 1.50 s);
the overshoot is bounded by the 0.05 s polling tick and is largest at the lowest
real-time factor, as expected. Under the previous wall-clock window the 0.5377
mission would have commanded about 0.54 s of motion per forward stage and
travelled roughly 0.061 m, failing the 0.08 m floor outright.

This was a measurement fault that fails healthy robots, so it inflated failure
rates identically across all four methods. It did not bias the method contrast,
but it did cost one of the nine workshop cells. The re-run evidence stands; the
published Neutral `geometry_coupled` `motion_qualification_failure` should be
read as an instrumentation artifact, not a platform reliability figure. The
"roughly one failure in four attempts" reliability estimate is withdrawn.

**The workshop abstract was accepted stating the withdrawn claim, and the
author has decided to leave its text unchanged.**
`paper/iros2026_codesign/main.tex` has a paragraph titled "A platform failure
that is not a planner difference" asserting that the gate "is intermittently
marginal, roughly one failure in four attempts, which is a platform reliability
limit", and a limitation sentence reading "Execution is also not reproducible
run to run, as the Neutral failure shows". Both are now known to be false: the
cause was a wall-clock command window in the evaluation harness, it is
deterministic in the real-time factor, and it is fixed. The corrected reading
is that the mission completes and the nine-cell count is 7/9 rather than 6/9.

Decision, 2026-09-20: do not edit the abstract. Acceptance was on the submitted
abstract, and the correction is recorded here rather than in the paper. This
document, not `main.tex`, is the current statement of what that cell means.
Anything derived from the abstract later -- a camera-ready PDF, an extended
version, a talk -- must carry the corrected reading rather than reproduce the
paragraph above. The re-run records themselves are unaffected, since no method
contrast depended on that cell, and the figures and results table are unchanged
by this decision.

Mission completion across the nine cells went from 5/9 to 6/9 and evaluator
collisions from 11 to 8.

Reproducibility gap found while summarizing, now closed at both ends.
`contact_phases.json` is consumed by
`paper/iros2026_codesign/summarize_results.py`, and
`make_contact_phases.py` now produces it by attributing each mission's
contacts to a phase from its execution outcome. That inference is sound for
the published records but cannot separate two contacts that occurred in
different phases of one mission, because the records carried only a scalar
`collision_count`.

`clearance_metrics` already distinguished contacts in order to count them and
then discarded the distinction. It now returns `contact_events`, one entry per
debounced contact carrying the module, the simulated time, and the clearance
that triggered it, with `collision_count` defined as its length. Contact-phase
attribution is a property of newly collected records rather than a consumer's
inference. The existing workshop records predate the field and still require
the outcome-based generator, so the paper's artifacts are unaffected; switch
the generator to `contact_events` when the records behind a figure have been
re-collected.

## Gate 0 fault matrix generalizes across layouts

`generate_fault_matrix_campaign` repeats the declared seven-case matrix across
layouts and independent `fault_seed` realizations, which is roadmap Gate 0
step 2. `generate_fault_matrix_design` is now the one-layout, one-realization
campaign and reproduces its recorded hash
`f1e172305fe3fb29f98a7fb54468fea16885090093a04111e1aaf0e73cfcf29e` exactly, so
the execution that already passed remains valid. `summarize_fault_matrix`
reports per-layout outcomes and additionally refuses a campaign with duplicated
fault seeds or a case missing from any layout. Freeze one with:

```
python3 -m modular_robot_benchmarks.engineering_qualification freeze-fault-matrix   --output config/fault_matrix_campaign.json --family reconfiguration_workspace   --layout-index 0 --layout-index 3 --layout-index 7   --method sensing_feasibility_coupled --realizations 2
```

The campaign is now frozen at `config/fault_matrix_campaign.json` with design
hash `de7a62328ff425bc42097cd8db5bf5ffb509c7695f398828e3c1a72ad9d9d489`: 42
trials, seven fault cases across layouts `reconfiguration_workspace-00`, `-03`,
and `-07` at two independent `fault_seed` realizations each, under
`sensing_feasibility_coupled`. Freezing it is a design decision and is recorded
here before execution so the layouts and seeds cannot be chosen after seeing
outcomes.

It has not been executed. The previously reported one-layout pass is
notes-only, since those records are gone, so this campaign establishes the
fault matrix from scratch rather than extending an existing result.

## The Gate 0 record sets are not retained

Every record set this document cites as Gate 0 evidence is absent from the
workspace, from the host, and from every checkout on this machine. `results/`
is listed in `.gitignore`, so raw terminal records were never under version
control, and no archive, tarball, or external copy was found. Nine of the
thirteen cited directories are gone:

`native_velocity_transition_latest22_raw`, `ack_roundtrip_latest34_raw`,
`disturbed_roundtrip_latest36_raw`, `rigidity_roundtrip_latest43_raw`,
`roundtrip_r15_repro_raw`, `gate0_smoke_fault`,
`retry_provenance_20260913_220228`, `qualification/roundtrip_20_raw`, and
`qualification/roundtrip_20_retry1_raw`.

Only `results/workshop_diagnostic/` and the three
`results/debug/neutral_geometry_repro_*` sets survive, because they were
produced most recently. The two campaigns this document describes as
"retained as a failed qualification" and as preserving 37/40 completed
missions are not retained. Gate 0's required artifact -- a machine-readable
qualification summary over those records -- cannot currently be produced, and
none of the numbers quoted from them above can be recomputed or audited. They
are reported here as prose only and should be read that way until re-collected.

This also settles what to do about the wall-clock command window. Every motion
gate measurement in those campaigns was taken through the truncating window, so
even had the records survived they could not establish a signed-motion margin:
their travel figures scale with whatever host load each run happened to meet.
Re-collection was required on correctness grounds independently of the loss.

Two things must change before the pilot, and the second is a decision:

1. Gate 0 signed-motion evidence has been re-collected post-fix against the
   same frozen design, whose hash
   `9c74a26749d466ba382caa6b4d227a033139ef24b8600a3d10ef90608a088b5e` still
   reproduces from `config/engineering_roundtrip_design.json`. See the section
   below; the audit summary is committed at
   `studies/gate0/roundtrip_20_postfix_audit.json`.
2. Retention needs a mechanism, not a convention. Terminal records run
   3.5--5.8 MB each, so the 432-trial confirmatory set will be roughly 1.9 GB
   and plain git tracking is not viable. Choose between git-lfs, an external
   archive addressed by recorded content hashes, or a reduced retained record
   schema, and write the choice down as an ADR. Until then the study rule
   "preserve failed runs and structured failure reasons" is enforced by nothing.

   The part of this that does not depend on the decision is in.
   `TrialStore.digests` and `TrialStore.campaign_digest` hash every terminal
   record file, and `qualify_roundtrip_batch audit` emits both beside its
   verdict. The audit summary is small enough to commit, so a record set stays
   identifiable after its bulk is gone: a re-collection can be shown to differ
   from the set a claim was computed on, and a restored archive can be shown
   to be the original. Had this existed, the lost campaigns would at least be
   provably lost rather than merely absent. Commit the audit summary for every
   campaign from here on.

## The 20-run round-trip gate passes

`results/qualification/roundtrip_20_postfix_raw` is the first Gate 0 round-trip
campaign to pass. All 20 frozen runs produced terminal records, all 20
completed, and the auditor returned no failures: two transitions each, the
compact-narrow-compact commit sequence, final `compact_diff`/`READY`, no
unrecovered fault, both signed-motion gates, all-pod rigidity, and 20 unique
realized disturbances spanning ground friction 0.718--1.085. Simulated
durations ran 151.4--153.3 s. It was collected at commit `5713195` against
design hash `9c74a267...` and its record set has campaign digest
`009e9efa1dfbed35327d3cc470b5a6e9703492678b4c934516ede7fd92f67886`. The audit
summary is committed at `studies/gate0/roundtrip_20_postfix_audit.json`; the
records themselves are not retained in version control, which is the open
decision above.

This is the machine-readable signed-motion artifact Gate 0 has always owed, and
the first time the gate's margin has been measurable rather than confounded by
host load. Over 40 gate measurements:

| Quantity | Bound | Observed | Worst margin |
|---|---|---|---|
| reverse travel | <= -0.08 m | -0.11520 .. -0.11069 | **+0.03069** |
| forward travel | >= 0.08 m | +0.11175 .. +0.11499 | **+0.03175** |
| forward lateral | <= 0.08 m | -0.00196 .. +0.00084 | +0.07804 |
| positive yaw | >= 0.10 rad | +0.24213 .. +0.47880 | +0.14213 |
| forward yaw | <= 0.15 rad | -0.02179 .. +0.00071 | +0.12821 |
| yaw coupling | <= 0.25 | +0.00309 .. +0.01419 | +0.23581 |

The travel floors are the binding constraints by roughly a factor of two and
everything else is nowhere near its limit, so an operational threshold should
be set from them. Set it from the coupling ratio rather than the absolute
yaw-translation bound, which never binds at the yaw these runs produce.

The campaign also tests load invariance directly rather than by argument,
because it happened to span real-time factor 0.7784--0.9894. Across that 27.1%
spread, forward travel varied by 2.9% and reverse by 4.1%, and the correlation
between real-time factor and travel is negative (-0.16 forward, -0.56 reverse)
where a wall-clock window would force it near +1. Under the previous window the
slowest run would have commanded about 0.0878 m of forward travel, a 0.0078 m
margin rather than 0.0317 m -- four times closer to failing, from mechanics
that did not change. Only two of the 40 measurements fall below 0.90 real-time
factor, so this campaign constrains the low-load end weakly; the 0.5377 replay
recorded above remains the evidence there.

Two limits on what this gate establishes. It is one layout
(`combined_constraints-00`) under one method, `feasibility_coupled`, so it
qualifies the platform rather than the study, exactly as the frozen design
intends. And it does not revive the lost campaigns: it stands on its own and
is not comparable to figures that cannot be re-audited.

## Resume here

Item 1 needs a planner sweep and then possibly a design decision; items 2 and 3
need container runs.

1. Run `check_planner_manipulation` over `studies/confirmatory/design.json` and
   read whether `combined_constraints` still manipulates across the twelve
   layouts the frozen design draws. Its golden fixture was re-selected to world
   seed 1 after the post-transition maneuver invalidated seed 8, but the family
   separates on only 4 of 12 sampled seeds; the roadmap Gate 2 entry records
   that sweep. `docking_observability` was unaffected at 12/12.

   The tool plans all four methods on every layout the design draws and counts,
   per declared contrast, how many layouts decide differently. It defaults to
   the search parameters `morphology_planner.ros_node` uses for missions. A
   decision is only owed if the count is low: the second confirmatory contrast
   (`sensing_feasibility_coupled` minus `feasibility_coupled`) is unidentifiable
   in any layout where the two methods choose the same site, so a family that
   rarely separates makes that contrast mostly noise no matter how many trials
   are run. Options would be to drop the family from the contrast's declared
   scope, re-draw its layouts, or strengthen the constraint the family encodes
   -- all of which must be decided and frozen before any outcome data exists.

   Note the check's own limit before reading too much into it: it drives the
   planner with each scenario's declared location-dependent observability,
   which is the manipulation the design assumes but not what missions currently
   supply. Separation there is necessary for the contrast, not sufficient.
2. Read the post-transition gate margin off the re-collected Gate 0 campaign.
   The measurement machinery is in: `audit_roundtrip_records` now emits
   `motion_margins`, giving each signed-motion quantity's observed range and
   its worst-case room before rejection, beside `real_time_factors` so that
   travel's independence from load is checkable rather than asserted.
   `MOTION_BOUNDS` mirrors the gate and a test drives the real predicate either
   side of every bound, so a margin cannot be reported against a limit the gate
   does not enforce. What is still missing is the campaign to read it from.

   Writing that binding test surfaced a property of the gate worth knowing
   before any threshold is set from it: the absolute yaw-translation bound
   (`translation_m <= 0.12`) cannot bind until yaw exceeds 0.12/0.25 = 0.48 rad,
   because the yaw-normalized coupling ratio (`<= 0.25`) is tighter below that.
   Engineering runs yaw about 0.45 rad, so in practice that bound has never been
   the constraint that rejects and its margin is not informative. Set the
   operational threshold from the coupling ratio and the travel floors.
3. Freeze and execute a multi-layout fault-matrix campaign. The generator,
   runtime injection map, and per-layout auditor are ready and host-tested.

Then proceed to the disjoint pilot without spending more submission time on
repeated 20-run startup certification. If throughput is still limiting, measure
trial wall time with the RGB-D camera disabled before deciding whether
perceived-obstacle work needs it at 15 Hz.

After mechanics pass, implement location-dependent perceived 3D obstacles and
observability, complete evaluator outcome/provenance streams, run a disjoint
pilot, freeze the operational effect threshold and conservative power grid, and
only then freeze and execute the confirmatory schedule.

## Last verified checkpoint

- Platform commit: `95a03c5` (`Pass the 20-run round-trip gate and commit its
  audit`), on `main` after the workshop branch was merged into it.
- Commanded maneuver windows advance on the simulated clock. Two replays at
  0.5377 and 0.8806 real-time factor both completed with zero collisions and
  both gates passing at the unloaded travel norm.
- The 20-run round-trip gate passes for the first time: 20/20 completed, no
  audit failures, 20 unique disturbances, collected at `5713195` with campaign
  digest `009e9efa...`. Worst signed-motion margins are +0.0317 m forward and
  +0.0307 m reverse travel.
- Every record set behind the previous Gate 0 claims is absent from this
  machine and cannot be re-audited.
- The two earlier engineering campaigns are described from notes only; their
  records are gone, so the 37/40 figure cannot be recomputed.
- Fault matrix: reported as 7/7 in one engineering execution, from notes only;
  those records are also gone. The refactored generator still reproduces its
  frozen design hash, so the campaign can be re-executed unchanged.
- Single-threaded executor regression: a frozen-design round trip completed with
  both motion and rigidity gates passing at mission real-time factor 0.95, and
  the identical seven-case fault matrix passed again with the same
  reconciliation outcomes in 405.8 s of summed trial wall time (519.4 s before
  the change).
- `python3 -m pytest -q`: 157 passed in 171.37 s.
- `colcon build --symlink-install`: all nine packages passed.
- `colcon test`: all five packages containing smoke tests passed; the remaining
  four contain no package-level tests.
- Documentation links/stale references and `git diff --check` passed.
- No simulator or bridge processes remained after verification.
