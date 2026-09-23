# ADR 0006: Location-dependent observability needs a physical cause

- Status: **accepted** as A1, 2026-09-22. A was chosen first and found
  unrealizable as written; see "Occlusion cannot be the physical cause".
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

## Occlusion cannot be the physical cause (measured 2026-09-22)

A was chosen, and then measured before implementation. Occluding a connector
sight line is not possible on this platform.

All four connector cameras are mounted at the core origin and differ only in
yaw (`sensing_node.py:60-65`), covering 4 x 1.8 rad, so a sight line is the
segment from the robot's own center to a pod. `compact_to_narrow` moves its
pods radially outward from 0.20--0.31 m to 0.35--0.74 m, which means the sight
line to a pod is very nearly the pod's own path.

Sampling each sight line against the real swept collision geometry (the core
box plus every collision part at every trajectory sample, 106 boxes) gives the
free runs available to an occluder:

| sight line | free run from core | candidate post | inside the robot's own footprint |
|---|---|---|---|
| `pod_0` (0.710, 0.200) | 14--41 cm | (+0.260, +0.073) | yes |
| `pod_1` (0.710, -0.200) | 14--54 cm | (+0.323, -0.091) | yes |
| `pod_2` (-0.710, 0.200) | 14--54 cm | (-0.323, +0.091) | yes |
| `pod_3` (-0.710, -0.200) | 14--41 cm | (-0.260, -0.073) | yes |
| `pod_4` (0.350, 0.200) | none >= 12 cm | -- | -- |
| `pod_5` (-0.350, -0.200) | none >= 12 cm | -- | -- |

Every free run lies between the relocating pods, at 14--54 cm from the site
center, and `narrow_tandem` has half-extent 0.88 x 0.33 m. So every placement
that would block a sight line is inside the robot's own post-transition
footprint: the target-footprint check and the ADR 0003 verification sweep would
reject the site for geometry, and `feasibility_coupled` would reject it too.
That collapses the contrast the manipulation exists to create.

The connector sensing geometry is entirely intra-robot. No external body can
occlude it without colliding with the machine.

## Revised options

- **A1. External fiducial dependence (the only remaining form of A).** Make the
  connector relative-pose estimate depend on an external workspace landmark.
  The `/fiducials/pod_N/pose` hook already exists in the sensing node
  (`sensing_node.py:56-59`) and nothing publishes it; visibility fusion already
  keys on `source == "fiducial"` (`sensing.py:112`). A sim-side publisher would
  model line of sight from a fixed marker to each pod, physical occluders in
  the world would block *that* line, and the planner's `sensing_provider` would
  predict visibility at a candidate site by raycasting the prior 3D map to the
  known marker pose. The occluder is then far from the robot, so there is no
  motion conflict, and the manipulation is both physical and location-dependent.
  Cost: a new sim-side node, marker placement in the generated worlds, sensing
  node changes inside a guarded autonomy package, a planner provider, scenario
  changes, a regenerated Gate 2 report, and re-qualification of the round-trip
  and fault-matrix gates that currently pass. It also changes the docking
  sensing architecture from a self-contained intra-robot design to one with an
  infrastructure dependency, which is a claim the paper would have to carry.
- **D. Withdraw the sensing contrast from the confirmatory design.** Unchanged
  from above, and now the cheap option rather than the fallback.

B and C are unchanged and still rejected: neither supplies an outcome
mechanism.

## Decision

**A1, decided 2026-09-22.** A is not implementable as written; B and C supply
no outcome mechanism; D was declined because it costs the sensing contrast.
The platform therefore gains an infrastructure assumption, which the paper must
state: precision docking depends on an external workspace fiducial station, and
a transition site is only well observed where that station can see the pods.

### Design

The manipulation acts on **covariance**, not on the visibility flag, so the
onboard connector cameras keep their present role and the existing
qualification worlds are unaffected.

- A **fiducial station** is a fixed workspace pose declared with the world. A
  pod is *station-observed* when the segment from the station to the pod is
  unoccluded by the declared 3D environment and within the station's range.
- Station-observed pods get the precise, low covariance that precision docking
  needs. Pods seen only by the onboard connector cameras keep visibility but
  carry a coarse covariance above the 0.015 gate, which is the physically
  honest reading of a short-range relative sensor with no absolute reference.
  `covariance_too_large` already exists as a rejection reason in both the
  planner predicate and `docking_acceptance`, so no new failure path is needed.
- **Backwards compatible**: a world that declares no station keeps today's
  behaviour exactly. Stations are opt-in per world, so the passing round-trip
  and fault-matrix gates and the Gate 0 qualification worlds are untouched, and
  only the confirmatory scenarios take on the new regime.
- **Sim side** (`modular_robot_sim`, not a guarded package): a node synthesizes
  `/fiducials/pod_N/pose` from simulator state and the declared occluders, via
  the hook that already exists at `sensing_node.py:56-59` and the fusion that
  already keys on `source == "fiducial"` (`sensing.py:112`).
- **Autonomy side**: the planner's `sensing_provider` predicts, for each
  candidate site, whether each moved pod would be station-observed there, using
  only priors it already holds — the station pose declared with the map, the
  `transition_environment` boxes, and the catalog's own transition geometry.
  No evaluator truth is consumed, so the guard at
  `test_evaluator_metrics.py:218` continues to hold.

Prediction and measurement deliberately share the geometry rule, so they cannot
drift apart, but they differ in their inputs: the planner predicts at candidate
sites from priors, and the station computes at the robot's actual pose from
simulator state. An evaluator check comparing predicted observability at the
chosen site against what the station actually delivered there therefore
audits the placement and the inputs, not the rule. That is weaker than two
independent implementations would be, and is the deliberate trade for a single
source of truth; a rule bug would satisfy the audit.

### The declared regions are not all physically realizable (measured 2026-09-22)

The mechanism is built and tested, and the first attempt to place the station
and screen in the confirmatory layouts failed. It is recorded here because the
reason is a property of the platform, not of the placement.

A site is rejected when **any** moved pod is shadowed, so the rejected set is
the screen's shadow **dilated** by the pods' reach, never equal to it. The
declared `observability_regions` were authored under a different model:
`ConfirmatoryScenario.sensing_for` gates on whether the *site centre* falls in
the region, and returns one verdict for all six pods. Realizing such a region
physically therefore needs the shadow **eroded** by the pods' reach. The first
placement expanded it instead, which dilated the rejected set twice over; the
Gate 2 report came back with `sensing_feasibility_coupled` unable to plan in 5
of 9 `docking_observability` layouts and 4 of 9 `combined_constraints` layouts,
against 0 in the committed report.

Eroding fixes the arithmetic but exposes a floor. `compact_to_narrow` docks its
pods at x offsets of +/-0.71 m, so:

| | declared | realizable |
|---|---|---|
| minimum rejected width in x | -- | **1.42 m** (2 x 0.71) |
| minimum rejected height in y | -- | 0.40 m (2 x 0.20) |
| `docking_observability` | 2.10 x 1.00 m | yes |
| `combined_constraints` | 0.60 x 0.50 m | **no**, needs 1.42 m in x |

The `combined_constraints` occlusion is 0.60 m wide. **No station geometry can
produce a rejected band that narrow on this platform.** That occlusion is the
one ADR 0004 authored specifically to restore the sensing contrast, under the
rule "occlude the band where the feasibility-aware site falls, keeping a
distinct visible feasible site", and ADR 0004 records what happens when it
stops binding: the sensing contrast is lost in that family, leaving
`docking_observability` as the only source, where the feasibility term never
binds.

### The floor is higher than 1.42 m, because the pod offsets are discrete

Screening the corrected placement refined this. The pods do not occupy a
continuum: `compact_to_narrow` docks them at x offsets of only
-0.71, -0.35, +0.35, +0.71 m. Dilating a narrow shadow by that set yields four
narrow slivers with gaps between them, not a band, and a site can sit in a gap.
The largest gap is the 0.70 m between -0.35 and +0.35, so a **contiguous**
rejected band needs a shadow at least that wide, and hence a declared region of
at least 0.70 + 2 x 0.71 = **about 2.12 m**.

Screened over the 9 non-neutral layouts of each family, heading bins 8,
epsilon 2.5:

| Configuration | separating | unplanned |
|---|---|---|
| `docking_observability`, declared 2.10 m, eroded | **9/9** | 0 |
| `combined_constraints` widened to 1.60 m | 0/9 | 0 |
| `combined_constraints` widened to 2.10 m | **9/9** | 0 |

1.60 m fails exactly as the discrete-offset argument predicts: its shadow is
0.18 m wide, its rejected slivers miss the feasibility-aware site, and sensing
chooses the same site as feasibility in all nine layouts. Both working
configurations sit at 2.10 m, just under the 2.12 m estimate, because at 0.1 m
grid resolution no site centre lands in the residual 2 cm gap.

In every configuration above, **every geometry and feasibility site is
identical to the committed Gate 2 report**, in both families and all 18
layouts. That is the empirical confirmation that the elevated screen is an
optical obstacle only, alongside the structural one: `methods.py:97` routes
only `sensing_feasibility_coupled` and `route_first_adaptation` through the
sensing predicate, so the other two methods never consult the provider at all.

So the correction restores `docking_observability` with no design change at
all, and `combined_constraints` needs its declared occlusion widened from
0.60 m to about 2.10 m.

That widening is a change to a declared manipulation of the frozen design. No
pilot or confirmatory data exists (0/432), so it is not a response to an
outcome, but it is an author decision: it takes the occlusion from a targeted
band around the feasibility-aware site to most of the staging band, which is a
coarser manipulation than ADR 0004 authored, even though the contrast it
exists to support still separates 9/9 with nothing unplanned.

### Adopted and regenerated (2026-09-22)

The author widened `combined_constraints` to 2.10 m and the placement is in.
`studies/gate2/confirmatory_manipulation_check.json` is regenerated and now
reports `station_observability_predicted_from_priors`:

| Contrast | Family | site | route | unplanned |
|---|---|---|---|---|
| sensing vs feasibility | `docking_observability` | 9/9 | 9 | 0 |
| sensing vs feasibility | `combined_constraints` | 9/9 | 9 | 0 |
| sensing vs geometry | `docking_observability` | 9/9 | 9 | 0 |
| sensing vs geometry | `combined_constraints` | 9/9 | 9 | 0 |

Matching the report it replaces, with all four methods agreeing in all six
neutral controls. Against that report **0 of 24 geometry and feasibility sites
changed**, which is the empirical confirmation that the elevated screen is an
optical obstacle only.

The station sits abeam the region it shadows. Anchoring it on the staging band
instead skews the projection where the two differ and costs
`combined_constraints` one separating layout (8/9); that was an implementation
deviation from the screened rule, corrected rather than accepted.

**Consequence for layout heterogeneity, recorded because the analysis is
layout-clustered.** The declared regions produced scattered sensing sites, in
`docking_observability` at y cells 13 to 23 either side of the route. A station
on the south wall produces a manipulation with a direction: every non-neutral
sensing site now falls between it and the route, at y cells 13 to 17, and the
sites cluster more tightly than before. The manipulation is more physically
coherent and less heterogeneous across layouts. Nothing in the analysis plan
depends on that heterogeneity today, but Gate 3 uses pilot data to characterize
layout heterogeneity, and this is a property of the design rather than of the
pilot.

### Before this is evidence

1. Regenerate `studies/gate2/confirmatory_manipulation_check.json`; the
   committed report describes layouts that will no longer exist.
2. Add the check that a declared observability region is reproduced by the
   mission sensing model at the sites it covers. It fails today by about 1600x.
3. Re-qualify the 20-run round-trip gate and the 42/42 fault matrix, which
   should be unaffected by backwards compatibility but must be shown so.
4. Consider binding the scenario generator's output into the frozen design
   envelope, so redefining a layout moves a hash at freeze time.
