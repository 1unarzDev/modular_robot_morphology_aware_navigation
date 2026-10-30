# Gate 3 pilot decision record

Drafted 2026-10-30 while the pilot was running and before its records were
read for anything but harness health. Sections marked **PENDING** are filled
from the generated artifacts named in them, by the rules stated here; nothing
else in this record may change once pilot data is read.

## What the pilot may and may not decide

| Decided by the pilot | Fixed before it, never revised by it |
|---|---|
| nuisance parameters for the power grid (`pilot_nuisance`) | smallest effect of interest, 0.18 |
| the confirmatory layout count, from the frozen grid | the two primary contrasts and their scopes |
| runtime projection for the confirmatory schedule | outcome definitions (analysis plan, "Outcome definitions") |
| measurement-validation findings and the fixes they need | layout generator 2 (ADR 0011), the methods, the deadline |

A fix the pilot motivates is allowed only if it corrects a defect for every
method alike and changes no scenario, method, threshold or outcome definition;
each one is an ADR naming the pilot as its source, followed by re-qualification
before the confirmatory freeze.

## Design

`studies/pilot/design.json`, kind `pilot_v2`, 144 trials, hash
`880af322987e...`, master seed 20261029; run at `6409b15` with container image
`sha256:2edb457d...`. The confirmatory master seed is 20260911, and
`study freeze-design --disjoint-from studies/pilot/design.json` must pass at
the confirmatory freeze.

## Measurement-validation findings

Found from harness-health checks while the pilot ran (no method contrast read):

1. **Route-first planning failures were labelled `docking_failure`.** Fixed at
   `6283df1`; the pilot's records keep their labels, and its failure taxonomy
   is recomputed from messages when reported.
2. **The controller could not see the raised shelf the planner routes around.**
   Every collision in the first 30 trials was `pod_0` clipping it on a diagonal
   approach. ADR 0012: a Nav2 keepout mask (`36e6c5f`), re-qualified by the
   generator-2 round trip (`config/roundtrip_v2_design.json`, hash
   `ace04ff0...`). **PENDING:** its audit.
3. The `unsafe_topology` outcomes of the geometry- and feasibility-coupled
   methods in `combined_constraints` are the manipulation working (docking
   refused at shadowed sites), not a defect.

## Nuisance parameters and layout count

**PENDING:** `pilot_nuisance --design studies/pilot/design.json --raw
results/pilot/pilot_v2_raw --power`, committed as
`studies/pilot/nuisance.json`. For each scope: the estimates, the declared
adverse values, and the smallest total layout count in 36, 48, ..., 120 whose
every grid cell meets 80% power on its Wilson lower bound. If no candidate
passes, the count is raised in steps of 12 until one does; the threshold never
moves.

## Runtime projection

**PENDING:** mean wall time per trial from `nuisance.json`, times
4 methods x 3 replicates x the chosen layout count, with the infrastructure
retry rate.

## Sign-off

**PENDING:** the decisions above, dated, by the author or under the author's
standing delegation, before `confirmatory_v2` is frozen.
