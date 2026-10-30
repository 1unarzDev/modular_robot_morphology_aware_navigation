# ADR 0013: Transitions must be executable, not only feasible

- Status: **accepted** (2026-10-30, from the Gate 3 pilot; a new pilot and
  Gate 0 re-qualification on generator 3 are owed before the confirmatory freeze)
- Date: 2026-10-30

## Context

The generator-2 pilot (`studies/pilot/design.json`, 144 trials at `6409b15`)
ran clean as a harness and found the study could not show an effect as built.
In manipulated layouts every method failed nearly always, the treatment
included (0/8, 0/8, 2/8 by family). The controls failing is the manipulation:
they choose shadowed sites and the docking gate refuses them. The treatment
failing is not: it transformed successfully at its observable site and then
could not drive the 1.76 m narrow body to the 0.70 m doorway.

That path -- transform off the door line, then turn the narrow body onto it --
had never been qualified. Gate 0's round trip used a neutral layout whose site
lies on the door line, and generator 1's sensing sites were already 0.1--0.4 m
off it. Reading the records found three method-agnostic defects, fixed first:
a snapped start reported in collision (`1c1a8c0`), transformations spending the
replan budget (`1c1a8c0`), and a controller blind to the raised shelf (ADR 0012).

With all three fixed, the generator-2 round trip of the treatment on manipulated
layouts still collided in 6 of 6 trials before it was stopped. Layout 01 shows
why: the treatment transforms 0.30 m off the door line beside the shelf, and the
planned narrow path is an exact S-turn that then runs 3.5 cm from it -- the
clearance ADR 0004 designed for straight driving. Regulated pure pursuit with a
0.8 m lookahead cuts that turn, and the nose swung into the shelf every time.

The planner treated any geometrically clear motion as executable. A long body
tracks a straight line closely and a turn loosely; the plan has to know that.

## Decision

1. **Turning needs a tracking margin.** Planner motion primitives that change
   heading must keep the footprint `TURN_CLEARANCE_M` = 0.10 m clear of
   everything (about the site-placement and localization error the pilot
   records show); straight primitives keep the exact check, so the 0.70 m
   doorway stays passable. It is part of the shared motion model, so every
   method and the route-first baseline get it alike.
2. **Start rooms long enough to turn in (layout generator 3).** With the margin,
   generator 2's 2.6--3.4 m start rooms leave the treatment no executable site
   at all: margins of 0.00/0.03/0.05/0.10 m planned it in 5/5/2/0 of the five
   re-qualification layouts. Generator 3 is generator 2 with the interior wall
   at 3.6--4.2 m and the arena 7.1--8.0 m wide; everything else is unchanged.
   On a screening sample every method planned in every manipulated layout and
   the treatment's site separated from both controls. Design kinds `pilot_v3`,
   `confirmatory_v3`, `engineering_roundtrip_v3`.

This extends ADR 0004's rule -- a manipulated layout must leave a distinct
visible feasible site -- to *executable*, and it extends ADR 0003's principle
-- feasibility covers every motion at the site -- to the drive away from it.

## Consequences

- The generator-2 pilot becomes the measurement-validation pilot that found
  these defects; its records stay as collected. Its nuisance estimates describe
  a stack and a layout population that no longer exist, so a new pilot on
  generator 3 (`pilot_v3`) supplies them, read by the rules fixed at `76ffe6e`
  and the stratified model of `7e24508`.
- Generator 1's manipulated layouts no longer plan under the margin, including
  the fault-matrix layouts `reconfiguration_workspace-03` and `-07`. Gate 0 was
  qualified on a stack that has since changed in the planner, navigator and
  costmaps, so the round trip and fault matrix are re-collected on generator 3
  before the confirmatory freeze. The neutral generator-1 layout behind the
  Gate 1 campaign still plans.
- The golden manipulation fixtures move to generator 3 with the same layout
  indices and world seeds, all of which still separate; none was re-selected.
