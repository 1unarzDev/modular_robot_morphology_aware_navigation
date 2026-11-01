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
it is accepted. **PENDING:** final counts
from `gate1_audit` and the analysis' `secondary_measures` (launch attempts,
infrastructure attempts, trials retried).

## Measurement-validation findings

**PENDING:** read after the pilot ends, by tracing every robot failure against
evaluator truth to its earliest cause, as for `pilot_v3`. A finding that is a
defect shared by all methods goes to an ADR and Gate 0; a finding that is the
manipulation working (for example docking refused at a shadowed site) is
recorded as such and changes nothing.

## Nuisance parameters and layout count

**PENDING:** `pilot_nuisance --design studies/pilot_v3b/design.json --raw
results/pilot/pilot_v3b_raw --power`, committed as
`studies/pilot_v3b/nuisance.json`. For each scope: the estimates, the declared
adverse values, and the smallest total layout count in 36, 48, ..., 120 whose
every grid cell meets 80% power on its Wilson lower bound. If no candidate
passes, the count is raised in steps of 12 until one does; the threshold never
moves.

## Runtime projection

**PENDING:** mean wall time per trial from `nuisance.json`, times
4 methods x 3 replicates x the chosen layout count, with the infrastructure
retry rate.

## Records

**PENDING:** `gate1_audit` committed as `pilot_v3b_gate1_audit.json`, records
retained and restore-verified with `retain_campaign_records` at
`studies/pilot_v3b/records/pilot_v3b_raw.tar.xz`.

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

**PENDING:** the decisions above, dated, under the author's standing
delegation, before `confirmatory_v3` is frozen.
