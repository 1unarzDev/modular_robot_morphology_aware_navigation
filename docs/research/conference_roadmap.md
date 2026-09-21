# Conference evidence roadmap

Updated 2026-09-12. Complete these gates in order. Engineering debug runs may
guide implementation but cannot satisfy pilot or confirmatory evidence gates.

## Submission claim

Evaluate joint route, morphology, and transition-site selection under 3D
reconfiguration feasibility and sensing uncertainty against route-first,
geometry-only, and feasibility-only alternatives. Keep broader system-first
claims out of the paper; the literature review defines the prior-art boundary.

## Gate 0: credible mechanics and fail-closed recovery

1. Qualify native wheel-velocity feedback on a detached pod, compact assembly,
   and post-transition narrow assembly.
2. Qualify `narrow_to_compact` with the same sensor-driven executor.
3. Pass 20 consecutive randomized compact-to-narrow-to-compact round trips.
4. Inject detach, relocation, latch, stale observation, cancellation, commit,
   and partial-topology faults. Every incomplete topology must inhibit assembled
   motion until explicit reconciliation.

Required artifact: machine-readable qualification summary with signed yaw,
straight/reverse travel, cross-coupling, docking errors, observed topology, and
failure-matrix results. Current state: **not satisfied, and being
re-collected**. The records behind every previous Gate 0 claim are gone --
`results/` is gitignored and no archive exists -- so the required artifact
cannot be produced from them and none of their figures can be audited. They
would have needed re-collection regardless: every signed-motion measurement in
them was taken through a command window bounded by the wall clock while the
robot moves in simulated time, so their travel figures scale with host load
rather than with mechanics. That is fixed, and a clean campaign against the
same frozen design is the current work. See `docs/current_status.md` for the
retention finding and the storage decision it forces.

Detached and compact native-feedback gates pass. Low-friction pod casters and an
explicit secondary wheel-friction coefficient produce two repeated engineering
missions whose compact-to-narrow yaw gates pass and whose planned reverse
transition gates also pass. A former observer ledger gap is repaired: a
subsequent run explicitly ends `compact_diff`, `READY`, with both
transition motion gates passing. Verify that randomized friction and initial-pose
seeds affect the plant, then begin the 20-run round-trip gate with per-pod
rigidity checks.

One applied-disturbance engineering mission now passes with randomized ground
friction and x/y/yaw initialization recorded identically in its world manifest,
trial provenance, Gazebo spawn pose, and AMCL inputs. This closes the single-run
wiring check. It does not close the reliability gate or establish measurable
between-seed plant sensitivity. A subsequent frozen-harness validation also
passes all-pod SE(3) rigidity after both transitions, with 61 synchronized pose
samples per pod in each motion window. The 20-run design is frozen at hash
`9c74a26749d466ba382caa6b4d227a033139ef24b8600a3d10ef90608a088b5e`.
The first independent campaign produced 19 completed missions and one startup
infrastructure failure caused by a missed controller lifecycle observation, so
Gate 0 did not pass. Fresh local-costmap output now supplies an additional
active-controller acknowledgement, and the exact failed seed passed on replay.
Both that campaign and its retry are described here from notes only; their
records are not retained and cannot be re-audited. The replacement campaign
must stand on its own rather than be compared against them.

The audit now reports `motion_margins` and `real_time_factors` beside its
verdict, so the replacement campaign yields the signed-motion margin Gate 0
owes and shows the load spread it was measured across.

The declared fault matrix injects detach, stale observation, cancellation,
relocation, latch, partial-topology, and manager-commit failures into the first
`compact_to_narrow` transition. One engineering execution passed all seven
cases: each ended `unsafe_topology`/`RECOVERY_REQUIRED`, a post-terminal body
velocity stimulus produced zero pod commands and zero core motion, and explicit
reconciliation returned `READY` only for complete `compact_diff` or
`narrow_tandem` topologies while refusing every partial topology. This is one
layout and one seed per case, not reliability evidence.

The next conference-evidence phases are ordered as follows:

1. Run the frozen 20 trials and require the full transition, morphology acknowledgement,
   motion, recovery-state, rigidity, and disturbance-uniqueness contract for
   every run. Archive failures rather than replacing them.
2. Repeat the declared fault matrix across layouts and `fault_seed` realizations,
   retaining the requirement that actual topology controls recovery and assembled
   drive remains inhibited until a valid `READY` commit.
   `generate_fault_matrix_campaign` and the per-layout auditor now exist and
   preserve the single-layout design hash that already passed; freezing and
   executing a multi-layout campaign is the remaining work.
3. Apply `sensing_seed` to sensor noise/dropout/occlusion and `fault_seed` to
   declared executor faults. Add location-dependent perceived 3D transition
   volumes and connector visibility without exposing evaluator truth to autonomy.
4. Complete evaluator-only synchronized clearance, collision, localization RMSE,
   transition prediction/outcome, and energy-proxy streams; then run a disjoint
   pilot to set nuisance ranges and the operational smallest effect of interest.
5. Freeze the power-grid decision, update the layout count if needed, and run the
   immutable paired confirmatory design. Only accepted terminal records feed the
   predeclared layout-clustered bootstrap, sign-flip tests, Holm correction, and
   hierarchical logistic sensitivity analysis.

## Gate 1: complete measured outcomes

Record and coverage-audit synchronized evaluator-only collision and 3D
clearance, localization error, predicted transition probability and outcome,
mechanical work, docking attempts, recovery actions, failure cause, process
health, and source/container/configuration hashes. Add suspension state while
diagnosing the active plant blocker.

Required artifact: controlled success, collision, localization-loss,
docking-failure, and timeout records plus an audit that rejects missing or
misaligned telemetry. Ground truth consumption by autonomy must be mechanically
audited and rejected.

## Gate 2: planner manipulation checks

Bind location-dependent 3D obstacles, visibility, and localization covariance
derived from simulated observations to every candidate transition site. Freeze
targeted cases for workspace feasibility, docking observability, and combined
constraints, plus neutral controls.

Required artifact: deterministic report for all four methods showing selected
route/morphology/site, cost terms, and structured rejection reasons. Each
ablation must differ only in its declared information and agree in neutral
controls.

Recorded design decision, 2026-09-19. Sweeping the post-transition
verification maneuver (ADR 0003) invalidated the `combined_constraints` golden
layout at world seed 8: under the corrected model `feasibility_coupled` and
`sensing_feasibility_coupled` both choose (18, 14), because the site
feasibility now prefers is one where the sensing constraint does not bind.

A sweep of world seeds 1--12 at layout index 1 measured how robust each family
is to the model change:

| Family | Seeds separating before | After |
|---|---|---|
| `docking_observability` | 12/12 | 12/12, identical sites |
| `combined_constraints` | 8/12 | 4/12 (seeds 1, 3, 9, 11) |

`docking_observability` is unaffected. `combined_constraints` was already the
fragile family and the added term narrowed it further. The golden fixture moved
to world seed 1, which separates under both the old and the corrected model and
by the widest margin, so it is not tuned to either. No pilot or confirmatory
data exists (0/432), so this re-selection cannot be a response to an outcome.

Remaining risk: `combined_constraints` separates on only a third of sampled
seeds. `check_planner_manipulation` measures this directly, planning all four
methods on every layout a frozen design draws and counting how many decide
differently under each declared contrast. Run it over
`studies/confirmatory/design.json` before relying on the family for the sensing
contrast: the `sensing_feasibility_coupled` minus `feasibility_coupled`
contrast is unidentifiable in any layout where those two methods choose the
same site, so a low count makes the contrast noise regardless of trial count.

The check drives the planner with each scenario's declared location-dependent
observability. Missions do not yet supply that -- the planner node consumes
live `RelativePoseEstimate` sensing -- so separation in the report is a
necessary condition for the contrast rather than a sufficient one, and closing
roadmap Gate 0 item 3 is what would make the two agree.

## Gate 3: unattended disjoint pilot

Run complete randomized paired blocks under isolated Gazebo/ROS processes.
Retain all robot failures and original infrastructure-failure audit entries.
Use pilot data for runtime, nuisance parameters, measurement validation, and
layout heterogeneity. Define the smallest operationally useful completion gain
from engineering needs rather than the observed method contrast.

Required artifact: immutable pilot records, completeness report, provenance,
and a signed design-decision record. Pilot layouts and seeds must not appear in
the confirmatory schedule.

## Gate 4: prospective power decision and freeze

Run the predeclared nuisance-grid power simulation for the 36-layout all-family
contrast and 24-layout sensing-family contrast at alpha 0.025. Require every
grid cell's 95% Wilson lower bound to reach the target; increase simulation
draws for Monte Carlo precision and independent layouts if necessary.

Required artifact: archived power-grid JSON, effect-threshold rationale, final
trial count, frozen design hash, source revision, container digest, maps,
parameters, manifests, and analysis version.

## Gate 5: confirmatory collection and analysis

Collect complete balanced blocks without inspecting method outcomes. Run the
layout-balanced completion contrasts, Holm-adjusted sign-flip tests, cluster
intervals, deadline-penalized time, and the frozen hierarchical logistic
sensitivity model only after integrity checks pass. Report null, negative, or
mixed results without changing the design.

Required artifact: immutable raw records, audit log, deterministic analysis
JSON/CSV/Markdown/SVG outputs, calibration and failure analyses, and a
one-command reproduction check.

## Gate 6: submission artifacts

Produce synchronized videos for representative success, baseline failure,
sensing-aware site choice, reconfiguration, and recovery. Overlay route,
morphology, candidate/rejected sites, uncertainty, sensing visibility, topology,
execution state, and predicted costs/risks without hiding failures.

Required artifact: paper-ready technical narrative, bounded novelty claim,
limitations, generated figures/tables, demonstration videos, and artifact guide.

## Deferred work

Ackermann steering, omni mechanics, crawler locomotion, articulated bodies,
stacking, elevated redocking, and physical hardware validation are outside the
confirmatory study. Add them only after the two-morphology experiment is
complete or if the research question is formally redesigned before pilot data.
