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

Wheel suspension resolved compact-assembly contact loading: detached signed yaw
is +1.0595/-1.0590 rad with +0.7089/-0.7090 m forward/reverse travel; compact
signed yaw is +0.6748/-0.6614 rad with at most 0.0139 m translation and 0.7501 m
straight travel.

`results/debug/suspension_align06_latest15_raw` completed all six relocations,
committed the transition, and returned to `READY`. Post-transition signed yaw
was only +0.01284/-0.01319 rad, so the motion gate correctly inhibited assembled
drive. Diagnostics implicated update-rate position differencing in wheel-speed
feedback. The Gazebo plugin now reads native `JointVelocity`, falling back to
finite differences if unavailable. Focused tests and compilation pass; runtime
qualification remains outstanding.

## Resume here

Run these gates sequentially using native velocity feedback:

1. Detached-pod signed yaw and forward/reverse qualification.
2. Compact-assembly signed yaw and straight-motion qualification.
3. Same-seed compact-to-narrow transition followed immediately by the narrow
   motion gate.

Stop at the first failure and diagnose from evaluator-only telemetry. If the
third gate fails, add suspension travel and velocity diagnostics to distinguish
wheel unloading from feedback error. If all three pass, qualify
`narrow_to_compact`, then run 20 randomized consecutive round trips and the
declared fault-injection matrix.

After mechanics pass, implement location-dependent perceived 3D obstacles and
observability, complete evaluator outcome/provenance streams, run a disjoint
pilot, freeze the operational effect threshold and conservative power grid, and
only then freeze and execute the confirmatory schedule.

## Last verified checkpoint

- Platform commit: `d77c2a7` (`Qualify bounded self-mobile pod mechanics`).
- `python3 -m pytest -q`: 90 passed in 161.95 s.
- `colcon build --symlink-install`: all nine packages passed.
- `colcon test`: all five packages containing smoke tests passed; the remaining
  four contain no package-level tests.
- Documentation links/stale references and `git diff --check` passed.
- No simulator or bridge processes remained after verification.
