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
`narrow_to_compact`, but the observer's final topology history ends at
`narrow_tandem` revision 25 despite `READY`; reverse-transition qualification
therefore remains open until final committed morphology is observed explicitly.

## Resume here

Make mission completion wait for and verify the committed target morphology
after every transition. Then qualify `narrow_to_compact` explicitly from a
narrow initial state, add per-pod post-latch rigidity checks, and run 20
randomized round trips followed by the declared fault-injection matrix.

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
