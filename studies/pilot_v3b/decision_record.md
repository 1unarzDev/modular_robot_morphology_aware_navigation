# Gate 3 pilot decision record: `pilot_v3b`

Drafted 2026-11-01 while the pilot was running (90 of 144 records) and before
its records were read for anything but harness health. Sections marked
**PENDING** are filled from the generated artifacts named in them, by the rules
stated here; nothing else in this record may change once pilot data is read.
It follows `studies/pilot/decision_record.md`, whose rules carry over
unchanged.

## What the pilot may and may not decide

| Decided by the pilot | Fixed before it, never revised by it |
|---|---|
| nuisance parameters for the power grid (`pilot_nuisance`) | smallest effect of interest, 0.18 |
| the confirmatory layout count, from the frozen grid | the two primary contrasts and their scopes |
| runtime projection for the confirmatory schedule | outcome definitions (analysis plan, "Outcome definitions") |
| measurement-validation findings and the fixes they need | layout generator 3 (ADR 0013), the methods, the deadline |

A fix the pilot motivates is allowed only if it corrects a defect for every
method alike and changes no scenario, method, threshold or outcome definition;
each one is an ADR naming the pilot as its source, followed by re-qualification
(Gate 0) before the confirmatory freeze. The pilot's own effect estimate is
recorded for runtime and measurement validation only and never enters the
power grid.

## Design

`studies/pilot_v3b/design.json`, kind `pilot_v3`, 144 trials (6 layouts per
family x 2 replicates x 4 methods), hash `482ccb77...`, master seed 20261107;
run at `672b2f0` with container image `sha256:2edb457d...`, the full container
suite (331) passing first. Planner manipulation check committed before any
trial on the planner it ran with (`manipulation_check.json`). The confirmatory
master seed is 20260911, and `study freeze-design --disjoint-from` must pass
against this design and every earlier one at the confirmatory freeze.

## Harness health while running

Observed without reading any method contrast: one commit and image throughout;
at 90 of 144, two trials needed one infrastructure retry each and none ended
as an infrastructure failure. `combined_constraints-00-r1-geometry_coupled`
timed out at readiness (lifecycle managers not yet active) and ran on its
second launch. `docking_observability-01-r0-route_first_adaptation` was
flagged for a DDS transport failure on a launch whose planner had already
reported no feasible adaptation of its fixed route; the retry reproduced the
same planning failure, so the flag changed no outcome. As the analysis plan
requires for a transport-flagged trial, that retry is read by hand here before
it is accepted. Final: 3 of 144 trials needed one
infrastructure retry (`nuisance.json` runtime), none ended as an
infrastructure failure, and no stack process survived the batch.

## Measurement-validation findings

Read 2026-11-01 after the pilot ended (144/144 at `672b2f0`, `study status`
analysis-ready, no integrity error), by tracing every robot failure to its
earliest cause. A finding that is a defect shared by all methods would go to an
ADR and Gate 0; a finding that is the manipulation working is recorded as such
and changes nothing. **No platform defect was found: every robot failure is the
mechanism its layout was built to exercise.**

| Method | Family | Outcome | Cause |
|---|---|---|---|
| `geometry_coupled` | `reconfiguration_workspace` | 8 `collision` | the 4 manipulated layouts x 2 replicates; the post-transition verification maneuver it does not sweep met the shelf (contact 69--83 s), drive inhibited |
| `geometry_coupled`, `feasibility_coupled` | `docking_observability`, `combined_constraints` | 8 `unsafe_topology` each per family | the 4 manipulated layouts x 2 replicates; docking refused at a site the station cannot see |
| `sensing_feasibility_coupled` | `combined_constraints` | 2 `planning_failure` | layout 03, the one the committed manipulation check showed it cannot plan, kept and scored against it |
| `route_first_adaptation` | all | 34 `planning_failure`, 2 completed | its fixed route admits no feasible transition, in neutral controls too |

Every neutral control completed under all three coupled methods (12 of 12
each), and the two replicates of every layout agree. `gate1_audit` flags the
treatment's two planning failures for "mechanical work not measured", which is
how it reads a trial that never moved, not a missing measurement; its
`gate_passed` is false only because no localization-loss or timeout outcome
occurred, a Gate 1 coverage criterion that is not a pilot criterion.

Finding for the paper, changing nothing in the design: `route_first_adaptation`
fails to plan in neutral controls as well, because the fixed shortest route
reaches the doorway with no site that admits the transition. It is in neither
primary contrast; the paper must report it as a weak rung in this population
rather than let its failures read as an effect of the information it lacks.

The pilot's effect estimates (treatment 34/36 completed against
`geometry_coupled` 12/36 and `feasibility_coupled` 20/36) are recorded for
measurement validation only and never enter the power grid.

## Nuisance parameters and layout count

**A defect in the power tool, found by its first reading and fixed before the
count was taken (2026-11-01).** The frozen reading at `672b2f0`
(`pilot_v3b_nuisance_672b2f0.json`) found no passing count: the worst cell's
lower bound was 0.04 at 36 layouts and 0.49 at 120 (all families), 0.004 and
0.19 (sensing families). The pilot's outcomes are all-or-nothing by layout
(neutral 1.0, manipulated control 0.0, replicates identical), so the layout SD
fit ran to 5.0 and its adverse value to 10. The stratified power model applied
the smallest effect of interest as a fixed logit gap at the median manipulated
layout; at those spreads its simulated layout-averaged effect was 0.075 and
0.041, not the declared 0.18, so the grid measured power for a fraction of the
effect the analysis plan defines (a layout-balanced difference in completion
probability). The model's docstring and its test both state the averaged
effect as the intent; the test checked it only at SD 0.5, where the defect does
not show. Fixed at `2d3df12`: the gap is solved so the averaged effect equals
0.18 at every spread (test at SD 0, 2, 5 and 10). The effect, the grid's
values, alpha, the target and the search are unchanged, and the fix depends on
the layout spread alone, never on the pilot's effect estimate. The paired-noise
estimate is 0.0 because the estimator is unidentified when layouts are
deterministic (shared and independent agreement both 0.48); it is kept as
declared, and at these spreads pairing has little effect on power.

Read on the fixed tool at `2d3df12` (`nuisance.json`, the frozen procedure
otherwise unchanged): in both scopes the neutral rate is 1.0, the manipulated
control rate 0.0, the layout logit SD 5.0 and the paired-noise fraction 0.0
(unidentified, see above). Frozen grids: neutral {0.862, 1.0} and manipulated
control {0.0, 0.138} for all families, {0.806, 1.0} and {0.0, 0.194} for the
sensing families; layout SD {5, 10}; pairing {0}. The all-family scope passes
at the floor, 36 layouts (worst-cell lower bound 0.814); the sensing scope
needs 40 of its own (24: 0.423, 32: 0.727, 40: 0.873), which is two thirds of
a 60-layout total. **Confirmatory layout count: 60, 20 per family, 720
trials.**

## Runtime projection

Mean mission wall time 114 s (max 354 s) excludes launch and teardown; the
pilot ran about 15 trials an hour end to end while its host was awake, so 720
trials take about 48 hours, plus about 2% infrastructure retries.

## Records

`gate1_audit` committed as `pilot_v3b_gate1_audit.json`; all 144 records
retained and restore-verified against its digests with
`retain_campaign_records` at `studies/pilot_v3b/records/pilot_v3b_raw.tar.xz`
(22.2 MB, campaign digest `dc9b590f...`, `pilot_v3b_retain.json`).

## Confirmatory freeze, decided before the pilot is read (2026-11-01)

Taken under the author's standing delegation at 96 of 144 records, with no
pilot outcome read; none of it depends on the layout count.

- `confirmatory_v3`: kind `confirmatory_v3` (layout generator 3), 3
  replicates, all four methods, the declared confirmatory master seed
  20260911, layouts per family = the grid's total / 3.
- **Disjointness set:** every committed frozen design (`studies/*/design.json`,
  `config/*design*.json`, `config/*campaign*.json`, `config/engineering/*.json`)
  except the superseded generator-1 `studies/confirmatory/design.json`
  (`e9115d54...`). A dry freeze in a scratch directory showed it shares
  sensing, friction and fault seeds with any design at the same master seed.
  Disjointness exists so that no confirmatory seed has an observed outcome, and
  that design never ran a trial (confirmatory evidence 0/432; only the planner
  manipulation check, Gate 2, ran on it, which observes no outcome). Keeping
  the declared master seed is preferred to choosing a new one. Every other
  design passes at 12 and at 40 layouts per family.
- **Manipulation check:** layouts are prefix-stable in the count (the first 12
  per family at 40 per family equal the 12-per-family design's), so
  `check_planner_manipulation` is run once on a 40-per-family superset and
  reported for the frozen prefix. It runs after the pilot ends so that it
  adds no load beside it, and it is committed before any confirmatory trial.

## Sign-off

2026-11-01, under the author's standing delegation: no platform fix is
needed; the power-tool defect is fixed with a test and disclosed; the layout
count is 60. `confirmatory_v3` is frozen at `studies/confirmatory_v3/
design.json` (hash `c939f126...`, 720 trials, master 20260911, disjoint from
the set above), and its manipulation check (`studies/confirmatory_v3/
manipulation_check.json`) passes before any trial: the treatment's site
separates from `geometry_coupled`'s in 41 of 45 manipulated layouts and from
`feasibility_coupled`'s in 26 of 30, every non-separating layout is one the
treatment cannot plan (4, kept and scored against it), and all 15 neutral
controls agree.
