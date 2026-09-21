# ADR 0004: Raised transition obstacles constrain driving, and the shelf leaves the route clear

- Status: **proposed** (not adopted; awaiting author decision)
- Date: 2026-09-21

## Context

The multi-layout Gate 0 fault matrix (13/42, `studies/gate0/fault_matrix_campaign_audit.json`)
showed that on non-neutral `reconfiguration_workspace` layouts the compact
robot drives into `raised_transition_shelf` and never reaches a transition
site. The shelf exists only in the 3D transition environment; the planner's 2D
grid and Nav2's map omit it, so routes pass through it.

Making every method treat the shelf as a driving obstacle, with the frozen
geometry, leaves **no path on any non-neutral layout** in either shelf-carrying
family. The shelf lies on the door-approach line (`center_y + 0.20`, the pods'
left track) and runs from just past the spawn to about 0.8 m before the wall.
The narrow robot has +/-0.02 m of lateral slack in the 0.70 m doorway, so it
must transition on the door line, and its left pods then sit inside the
shelf's span. The frozen worlds were solvable only because planning ignored
an obstacle the robot physically hits. No pilot or confirmatory data exists,
so this can be corrected without responding to an outcome.

## Decision (proposed)

1. `MorphologyAStar` takes `drive_obstacles`; a box whose underside is below
   a morphology's height blocks that morphology's footprint during traversal.
   `make_method_planner` passes the `environment` every method already
   receives, so all methods drive around the same obstacles. Transition
   validation is unchanged and remains the only place methods differ.
2. The shelf moves off the pod track into the band only the relocation sweep
   reaches, following the workshop post: centre `center_y + 0.415`, depth
   0.10 m (inner edge 0.365 m, outside the compact 0.36 m safety footprint and
   the physical robot's 0.2875 m, overlapping pod_4's sweep to 0.423 m).
3. The shelf keeps its door-side end (`band_x1`) but spans a fixed length
   `SHELF_LENGTH_M` instead of reaching back to the spawn, which leaves an
   earlier door-line site whose sweep clears it.

## Evidence (planner only, debug; not study data)

Screened over the 18 shelf-carrying non-neutral layouts drawn by
`config/fault_matrix_campaign.json` and `studies/confirmatory/design.json`
(heading bins 8, epsilon 2.5):

| `SHELF_LENGTH_M` | Solvable (geometry / feasibility / sensing) | geometry != feasibility site | feasibility != sensing site |
|---|---|---|---|
| frozen geometry, shelf blocks driving | sensing only tested: 0 of 10 sampled (indices 1, 2, 3, 5, 7 of both families, seed 8) | -- | -- |
| items 1-2 only (full-length shelf) | fault-matrix layouts 03, 07: geometry solvable, sensing **no path** | -- | -- |
| 0.4 | 18 / 18 / 18 | 18 | **0** |
| 0.6 | 18 / 18 / 18 | 18 | **0** |
| 0.8 | 18 / 18 / 18 | 18 | **0** |

The geometry contrast (the shelf manipulation) survives at every length:
`geometry_coupled` takes the direct site beside the door; the
feasibility-aware methods back off to an earlier site whose sweep clears the
shelf.

## Consequence that blocks adoption as-is

The **sensing contrast is lost at every length**. Gate 2 recorded
`feasibility_coupled` and `sensing_feasibility_coupled` separating in 9/9
non-neutral `combined_constraints` layouts. That separation came from the
feasibility site falling in the occluded lower lane (`center_y - 0.85` to
`- 0.45`). Under this proposal the feasibility site is on or near the door
line, outside the occlusion, so both methods choose it. Adopting items 1-3
alone would remove one of the design's two declared contrasts.

Restoring it needs a coupled change to `combined_constraints`' observability
region: occlude the band where the feasibility-aware site now falls, leaving
a distinct visible feasible site. That is a further change to frozen worlds
and is the author's decision.

## Options

- **A.** Adopt items 1-3 with a rule-derived length, and redesign the
  `combined_constraints` occlusion by a declared rule; re-run the Gate 2
  manipulation check on all 24 layouts before freezing.
- **B.** Adopt items 1-3 and accept that `combined_constraints` no longer
  identifies the sensing contrast; independent sensing information would then
  come only from `docking_observability`, where Gate 2 found the feasibility
  term never binds. This weakens the study.
- **C.** Reject; keep the frozen worlds and record that non-neutral shelf
  layouts cannot be completed physically.

Adoption of any option changes `studies/confirmatory` worlds without changing
the design hash (worlds are generated from seeds at run time), so the ADR
itself is the record that must precede any pilot data.
