# ADR 0006: Location-dependent observability needs a physical cause

- Status: **proposed** (awaiting author decision)
- Date: 2026-09-22

## Context

Roadmap item 3 asks missions to supply location-dependent perceived 3D
transition volumes and connector visibility without exposing evaluator truth to
autonomy. The planner already has the hook: `CoupledTransitionPolicy` accepts a
`sensing_provider(transition, state)` and calls it per candidate site
(`transition_policy.py:311-315`), and the Gate 2 manipulation check drives it
with `ConfirmatoryScenario.sensing_for` (`manipulation_check.py:78`). Missions
instead pass a flat dict holding the last live `RelativePoseEstimate`
(`ros_node.py:172`), so every candidate site is scored with whatever sensing
the robot measured at its current pose.

Wiring the hook to a location-dependent source is a small change. It is also
not sufficient, and the measurement below is why.

## Measurement

`docking_observability` layout 1 (world seed 1) declares a poor-observability
band x 0.00--2.10, y 1.45--2.45 with `connector_visible=False` and
`covariance_trace=0.04`. That is 2.7x the 0.015 gate, so the sensing predicate
rejects every site in the band. The same layout contains **zero** transition
obstacles: `make_confirmatory_scenario` sets `obstacles = ()` and
`regions = (poor_region,)` for this family (`confirmatory_scenarios.py:165-167`),
and `export_confirmatory_sdf` only ever emits bodies for
`scenario.transition_obstacles` (`sdf_export.py:136`). Nothing in the world is
placed to cause the declared band.

Evaluating the mission's own sensing model at the band center (1.05, 1.95), for
the six pods `compact_to_narrow` moves, at their docked poses:

| pod | range (m) | in frustum | covariance trace | sensing gate |
|---|---|---|---|---|
| `pod_0`, `pod_1`, `pod_2`, `pod_3` | 0.738 | yes | 2.48e-05 | accepts |
| `pod_4`, `pod_5` | 0.403 | yes | 9.50e-06 | accepts |

The four connector sensors are `logical_camera` (near 0.05, far 1.2, hfov 1.8;
`sensor_core/model.sdf:25-43`). A Gazebo logical camera reports every model
whose pose falls inside its frustum and performs no occlusion test, and
`sensing_node.py:148-151` derives covariance from range alone
(`1e-6 + 1e-5 * range^2`), with an in-code note that noise injection and
covariance must change together. Observability in this stack is therefore a
function of range and nothing else. **No object placed anywhere in the world can
make `connector_visible` false or push the covariance trace above 0.015 inside
that band.**

The declared value and the achievable value differ by a factor of about 1600,
in the opposite direction from the manipulation.

`combined_constraints` is in the same position. It does place a physical body
(the raised shelf), but its occlusion region is authored where the
feasibility-aware site falls rather than derived from the shelf's optics
(`confirmatory_scenarios.py:170-181`), and the shelf could not occlude a
logical camera in any case.

## Why wiring alone is not enough

The same two criteria gate the planner's site choice
(`transition_policy.py:365-370`) and the executor's docking acceptance
(`sensing.py:146-151`): `connector_not_visible` and `covariance_too_large`.
That shared predicate is the mechanism by which choosing a poorly observable
site is supposed to cost a mission its transition, and it is already wired end
to end.

If missions take observability from a declared prior, `sensing_feasibility_coupled`
will avoid sites at which the executor would in fact have docked without
difficulty. The decision contrast would appear in the records; the outcome
contrast would have no cause. A confirmatory result on the sensing contrast
would then measure only that one planner refuses sites for a reason the
simulation never realizes — which is a property of the planner's inputs, not of
morphology-aware navigation.

This matters more than it would otherwise because the roadmap already records
that in `docking_observability` the feasibility term never binds, so
independent information for `sensing_feasibility_coupled` minus
`feasibility_coupled` comes only from `combined_constraints`
(`conference_roadmap.md:160-163`). Both sensing-contrast families rest on
declared regions with no physical cause.

## Options

- **A. Physical cause plus occlusion-aware connector sensing (recommended).**
  Give the sensing-contrast layouts a physical occluder, and replace the
  frustum-only visibility test with one that consults geometry: either a
  visibility check in the sensing node against the same prior 3D environment
  the planner loads, or a rendered sensor. The planner's `sensing_provider`
  then predicts visibility at a candidate site by the same geometric rule,
  from the prior map and the connector-camera model — no evaluator truth, and
  the prediction is falsifiable against what the robot measures when it gets
  there. Cost: the largest change of the four, and it touches an autonomy
  package, so it needs the ground-truth guard re-checked
  (`test_evaluator_metrics.py:218`).
- **B. Declared observability prior.** The scenario manifest already serializes
  `observability_regions` (`confirmatory_scenarios.py:107`) and
  `load_environment_boxes` simply ignores the key. Load them in the planner
  node exactly as `transition_environment` is loaded and build the
  `sensing_provider` from them. Smallest possible change and exact parity with
  the Gate 2 report. Rejected as a confirmatory basis for the reason above: it
  manipulates the decision without manipulating the world, so the sensing
  contrast has no outcome mechanism.
- **C. Declared prior, audited against measured sensing.** B, plus an evaluator
  stream recording measured visibility and covariance wherever the robot
  actually travels, and an audit that the prior agrees with measurement where
  they overlap. Makes the prior falsifiable without making it physical.
  Overlap is limited to sites the planner chose to visit, so the audit is
  weakest exactly where the manipulation is strongest.
- **D. Withdraw the sensing contrast from the confirmatory design.** If
  observability cannot be made physical, `sensing_feasibility_coupled` versus
  `feasibility_coupled` is not testable on this platform, and the study reduces
  to the contrasts that are physically realized. Honest, and much cheaper than
  A, but it drops a core claim and is squarely an author decision.

## Consequences for the freeze

Any option that edits `confirmatory_scenarios.py` redefines what the frozen
design's layouts are. `design_hash` covers only trial assignments — schema,
kind, counts, seeds, methods, families, and per-trial fields
(`design.py:63-94`) — so a layout can be redefined under an unchanged design
hash. Per-trial `configuration_hash` does cover the world, manifest, and map
files (`mission_batch.py:203-216`), and `analysis.py:35` refuses a campaign
whose records disagree, so drift is caught within a campaign but not between
the freeze and collection.

Therefore, before adoption of A, C, or D:

1. Re-run `check_planner_manipulation` over `studies/confirmatory/design.json`
   and re-commit `studies/gate2/confirmatory_manipulation_check.json`; the
   existing report describes layouts that would no longer exist.
2. Add a check that a declared observability region is reproduced by the
   mission sensing model at the sites it covers, so this gap cannot reopen
   silently. It fails today, by a factor of about 1600.
3. Consider binding the scenario generator's output into the frozen design
   envelope, so redefining a layout moves a hash at freeze time rather than
   only in the per-trial records.

No pilot or confirmatory data exists (0/432), so changing layouts now is not a
response to an outcome. It is still a design decision that must be frozen
before data exists, which is why it is recorded here rather than made as a side
effect of wiring the `sensing_provider`.

## Decision (proposed)

Adopt A. B is the change the roadmap item literally describes and would take an
afternoon, but it would buy a decision contrast that no outcome can follow, and
the confirmatory sensing result would not mean what the paper would say it
means. D is the correct fallback if A proves impractical on this platform, and
is preferable to B.
