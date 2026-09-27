# ADR 0008: The sweep margin is raised; the site tolerance cannot be

- Status: **accepted**, 2026-09-26 (tolerance raise attempted, measured and withdrawn the same day)
- Date: 2026-09-26

## Context

Two numbers were incompatible.

`site_position_tolerance` was 0.030 m. The controller servos the AMCL estimate,
because simulator ground truth is evaluator-only, and that estimate's own error
was 0.0296 m at the one successful alignment, 0.0221 m mean with 0.0647 m maximum
over that run, and 0.0516 m mean with 0.1021 m maximum over the sensing run. A
tolerance below the estimator's error asks for a placement the estimate cannot
resolve, so "reached the site" could not mean what it said, and the 1.3 mm margin
recorded from the one successful alignment was not a physical margin.

`post_transition_verification.margin_m` was 0.0, declared as "execution tolerance
is not yet modelled", so the planner checked the maneuver at the exact planned
site pose while the robot transformed wherever it actually stopped. That gap
existed at the old tolerance too: 0.030 m of tolerance plus 0.1021 m of estimator
error is 0.13 m of uncovered placement error against a margin of zero. Raising the
tolerance without raising the margin would widen a gap that was already open.

Raising the margin is bounded by a third constraint.
`test_clearance_disk_already_covers_the_verification_maneuver` requires the
verification sweep to stay inside the transition's planar clearance disk, because
the disk is what covers walls -- the 2D occupancy map cannot represent the 3D
obstacles the sweep checks.

## Measurement

The sweep's reach, as that test measures it:

| Transition | Reach at `margin_m` 0 | `swept_radius` | Headroom |
|---|---|---|---|
| `compact_to_narrow` | 0.9200 m | 1.0 | **0.0800 m** |
| `narrow_to_compact` | 0.4400 m | 1.0 | 0.5600 m |

`margin_m` inflates each box by the margin and reach grows exactly one-for-one,
measured across margins 0.0--0.09 (969 swept boxes throughout, reach
0.92 + margin to five decimals). So 0.08 m fills the existing disk to **exactly
zero headroom**, and anything larger needs the disk to grow -- and the disk is
what selects sites. Screened with `check_planner_manipulation` over the
frozen design, each candidate a full 24-layout run:

| `margin_m` | `swept_radius` | Separating, all 4 cells | Unplanned | Sites changed, of 96 | Golden fixtures |
|---|---|---|---|---|---|
| 0.00 | 1.00 | 9/9 | 0 | 0 (reproduces the committed report) | pass |
| **0.08** | **1.00** | **9/9** | **0** | **4** | **pass** |
| 0.13 | 1.05 | 9/9 | 0 | 4 | **fail** |
| 0.17 | 1.10 | 9/9 | 0 | -- | **fail** |
| 0.20 | 1.13 | 9/9 | 0 | -- | **fail** |
| 0.22 | 1.15 | 9/9 | 0 | 64 | **fail** |

The contrast counts alone would have accepted every candidate, including the
widest. The golden fixtures are what discriminate, and they say the opposite:

- From `margin_m` 0.13 upward the golden `combined_constraints` layout stops
  separating -- `feasibility_coupled` and `sensing_feasibility_coupled` both
  choose (12, 14). Inflating the boxes rejects the sites the sensing method used
  to be pushed toward, so the two methods converge.
- At `swept_radius` 1.15 the golden doorway plan fails outright with
  `NoPathError`. A 1.15 m disk needs a 2.3 m clear circle, which the 2.5 m-tall
  doorway fixture cannot hold; its comment records that it was sized for the
  1.0 m sweep.
- At 0.22/1.15, 64 of 96 site decisions move, for every method, because the disk
  is what selects sites in doorway-constrained worlds.

`reconfiguration_workspace` was screened separately because the fault matrix runs
there: under 1.15 m all three real methods still plan in all 12 layouts, and the
only unplanned decisions are `route_first_adaptation`, which fails by definition
in non-neutral layouts. So the fault matrix was not the binding constraint; the
golden fixtures were.

## Decision

**`post_transition_verification.margin_m` 0.0 -> 0.07 m. `swept_radius` unchanged
at 1.0. `site_position_tolerance` stays at 0.030 m.**

The tolerance raise was adopted at 0.07 m, executed, and withdrawn on the
evidence of the campaign it was meant to enable.

0.07 m for the margin is the ceiling less a safety margin: reach grows one-for-one
with the margin from 0.92 m inside a 1.0 m clearance disk, so 0.08 m would put the
wall check at exact equality and 0.07 m keeps 0.01 m of headroom. Equality is not
a margin. At 0.07 m the margin covers the 0.030 m tolerance plus 0.04 m of
estimator error, which is strictly better than the 0.0 it replaces, and it moves
no decision in the round-trip layout: that layout transitions at cells (20, 18)
and (48, 18) at both `margin_m` 0.0 and 0.07.

### Why the tolerance cannot be raised

The 20-run round trip was re-collected with the tolerance at 0.07 m and **failed
11 of 20** (`studies/gate0/roundtrip_20_fdir1_attempt1_failed_audit.json`,
retained). Every failure is the same event, and it is geometric:

- All 11 jam in a 24 cm band at x 2.315--2.559, y 1.981--1.993, yaw 0.033--0.067,
  after one transition, final morphology `narrow_tandem`/`READY`, no unrecovered
  fault.
- The doorway is at x 3.15 with a 0.70 m opening centred on y 1.95. The narrow
  body is 1.76 m long, so a core at x 2.32 has its nose at x 3.20 -- inside the
  doorway -- and its 0.66 m footprint leaves **0.02 m of slack per side**.
- The bodies sat about 0.034 m off centre, and a 0.06 rad yaw error adds 0.053 m
  of nose excursion over the 0.88 m half-length. Four runs recorded exactly
  0.0000 m clearance with 1--3 contacts; the rest scraped at 0.004--0.013 m. Nav2
  reported "Failed to make progress" 33 times.

Neither ADR 0009 mechanism fired -- 0 rotation-stall aborts, 0 abandoned
alignments -- and the margin moves no decision here, so the tolerance is what
changed and the doorway is what it broke. A robot permitted to stand 0.07 m off
the planned site starts the narrow leg further off centre than the doorway's slack
allows, with about 1.1 m to correct in, and a 1.76 m skid-steer on 0.08 lateral
friction does not correct that far.

**So the original defect stands and is not a tuning problem.** The doorway demands
a placement tighter than the estimator can resolve: 0.02 m of per-side slack
against an estimator whose error reached 0.1021 m. Raising the tolerance to match
the estimator makes the traversal fail; leaving it at 0.030 m means "reached the
site" continues to assert a precision the estimate does not have. That is a
platform limitation, recorded here rather than tuned away. The options are all
larger than this ADR: widen the confirmatory doorways so the narrow footprint has
real slack; add a pre-doorway re-centring maneuver that servos on something better
than AMCL; or accept and report the tolerance as nominal rather than physical.

## Consequences

**What is covered.** The sweep now covers the 0.030 m of position slack the
alignment law permits plus 0.04 m of the estimator's error. Four of 96 site decisions move, all
`sensing_feasibility_coupled` in `combined_constraints`, by one cell, and the
regenerated `studies/gate2/confirmatory_manipulation_check.json` keeps 9 of 9
non-neutral layouts separating on site and route in all four contrast cells with
0 unplanned.

**What is not.** The estimator's error beyond 0.04 m -- it reached 0.1021 m --
remains unmodelled, so a transformation can still run about 0.13 m from the
checked pose in the worst case seen. Closing that needs a larger margin, which
needs a larger clearance disk, which measured worse rather than better. Both open
items -- the uncovered estimator error and the unresolvable tolerance above --
point at the same scenario-design change: transition workspaces and doorways with
enough slack that the geometry stops being the binding constraint.

`margin_m` also inflates boxes linearly while yaw error grows with radius, so the
0.05 rad yaw tolerance contributes about 0.048 m of arc at the farthest swept
corner, inside the 0.07 m margin at that radius but not modelled as an angular
term.

The clearance-disk invariant keeps 0.01 m of headroom: reach is 0.9900 m against
a 1.0 m disk. That is deliberately small -- the margin is near its ceiling -- so
`test_clearance_disk_already_covers_the_verification_maneuver` is now the binding
guard on any change to the verification maneuver, and it will fail loudly rather
than let the wall check go stale.

One measurement was discarded getting here, and it was a scratch-harness bug
rather than a physical finding. The screening script patched the catalog by
replacing the literal `margin_m: 0.0`, which is a **prefix** of `margin_m: 0.08`,
so once the adopted value was in place the patch produced `margin_m: 0.088` and
reported a reach of 1.008 m -- an apparent 8 mm violation of the clearance disk
that did not exist. The candidate screen above predates the adoption and is
unaffected (the literal matched exactly then), and the reach figures here are
recomputed with the same code path the invariant test uses.

Raising the position tolerance does not undo what site alignment was added for.
The fault-matrix failure it fixed was a **heading** error -- the robot stopped
0.10 m short at yaw 0.02--0.06 rad when the planned site was at -45 degrees --
and `site_yaw_tolerance` stays 0.05 rad.

`design_hash` does not move, because it covers trial assignment only
(`design.py:63-94`). Per-trial `configuration_hash` covers the world and
manifest, and `analysis.py:35` rejects a mixed campaign, so drift is caught
within a campaign but not between a freeze and collection. Re-collect the 20-run
round trip and the fault matrix after this, so they qualify the geometry the
study will actually run; both are already superseded by ADR 0007. The first
re-collection attempt is the failed campaign described above and is retained.
