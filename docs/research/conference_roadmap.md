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
failure-matrix results. Current state: **blocked at post-transition narrow yaw**.

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
