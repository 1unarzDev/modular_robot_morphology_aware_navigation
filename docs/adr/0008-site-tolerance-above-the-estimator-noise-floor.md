# ADR 0008: Site tolerance above the estimator noise floor, with the largest sweep margin the geometry admits

- Status: **accepted**, 2026-09-26
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

**`site_position_tolerance` 0.030 -> 0.07 m, `margin_m` 0.0 -> 0.07 m,
`swept_radius` unchanged at 1.0.**

The tolerance is set equal to the margin, so the sweep covers exactly the slack
the alignment law permits. 0.08 m would be the arithmetic ceiling but puts the
clearance-disk invariant at exact equality (reach 1.00000 against a 1.0 m disk);
0.07 m keeps 0.01 m of headroom, which costs 0.01 m of coverage and buys back the
property that a small future change to the maneuver cannot silently invalidate
the wall check. Walls are a safety property and equality is not a margin.

This departs from the 0.12 m figure the decision was framed around, and the
reason is measurement rather than preference: at every margin large enough to
cover a 0.12 m tolerance, the sensing contrast stops separating in the golden
layout or the doorway plan disappears. Full coverage is not attainable by tuning
these three numbers.

The tolerance is still 2.3x the old value and above the 0.0296 m estimator error
recorded at the alignment that succeeded, so it asks for a placement the estimate
can resolve, which is the defect this ADR exists for.

## Consequences

**What is covered.** The sweep now covers the 0.07 m of position slack the
alignment law permits. Four of 96 site decisions move, all
`sensing_feasibility_coupled` in `combined_constraints`, by one cell, and the
regenerated `studies/gate2/confirmatory_manipulation_check.json` keeps 9 of 9
non-neutral layouts separating on site and route in all four contrast cells with
0 unplanned.

**What is not.** The estimator's own error on top of the tolerance -- up to
0.1021 m observed -- remains unmodelled, so a transformation can still run up to
about 0.18 m from the checked pose in the worst case seen. Closing that needs
roomier transition workspaces so the clearance disk can grow without erasing the
contrast, which is a scenario-design change and must not be made as a side effect
of tuning. It is the open item this ADR leaves.

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
study will actually run; both are already superseded by ADR 0007.
