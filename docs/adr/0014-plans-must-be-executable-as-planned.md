# ADR 0014: Plans must be executable by the controller as planned

- Status: **accepted in part** (2026-10-31, agent decisions under the author's
  standing delegation for pre-data design decisions; no `pilot_v3b` trial has
  run). Decisions 1--3 are adopted on their experiments; 4 awaits its own.
- Date: 2026-10-31

## Context

ADR 0013 required transitions to be executable, not only feasible, and added a
tracking margin to turning. Two engineering experiments on the generator-3
pilot's failures (`docs/current_status.md`, 2026-10-31) found the same class of
defect twice more, each a motion the planner approves that the controller
executes differently.

1. **Reverse driving.** `_motion_neighbors` offered the reverse step at the
   forward step's cost, and no RPP controller in the stack reverses. RPP turns
   the body around in place and drives forward instead. For the compact body
   that changed the heading the robot arrived at a site with, and the replan
   transformed it there; for the 1.76 m narrow body it meant a ~2.5 rad
   in-place turn the planner never swept, after which pure pursuit entered the
   planned bend from the wrong state, ran 11 cm off the route and was held by
   `collision_monitor` 1--2 cm from an obstacle. Every narrow failure traced in
   the turn-rate experiment begins with such a turn. Correcting the narrow turn
   rate (`yaw_effort_scale` 1.7) exposed this: the baseline's over-rotation had
   sometimes landed on a workable heading by chance.
2. **Driving beside a transition obstacle at zero margin.** Straight primitives
   keep the exact footprint check against raised obstacles, so routes run
   0.5 cm (planner footprint) from the shelf. The controller's view of the
   shelf is rasterized outward and its tracking error is about 4 cm, so RPP
   aborts on a route the planner approved. A finer keepout raster and local
   costmap removed three of four reproduced aborts but not the fourth, which
   came within 1.6 cm of the worst-case lethal edge: the remaining cause is
   tracking error, which only the plan can leave room for.

Enabling `allow_reversing` in RPP instead was rejected: RPP forbids it together
with `use_rotate_to_heading`, and the plans depend on in-place turns.

## Decision

1. **Forward traversal only.** Motion primitives move along the heading, never
   against it; heading changes stay in-place turns, which ADR 0013 already
   checks with its turning margin. It is the shared motion model, so every
   method and the route-first baseline get it alike.
2. **A traversal margin to drive obstacles.** Every traversal footprint keeps
   `DRIVE_OBSTACLE_CLEARANCE_M` = 0.03 m from raised drive obstacles, the most
   the generator-3 population tolerates; walls keep the exact check.
3. **The finer keepout raster**: the mask at 0.0125 m and the local costmap
   at 0.025 m, so the controller sees a transition obstacle within about one
   fine cell of its edge. Insufficient alone (three of four reproduced aborts
   cleared); adopted with decision 2.
4. **Narrow vertices executed in place.** Pending its experiment: the
   navigator splits a narrow path at its heading changes and servoes each
   vertex, instead of letting pure pursuit cut it. A shorter lookahead
   (0.35 m) and a planner rule demanding a body length of straight run-in
   before a tight passage were both measured and rejected: the first still
   left six of seven trials beyond 0.08 m of their route, the second made the
   treatment unplannable in 4 of 12 manipulated layouts where every control
   planned.

## Measurement before deciding

**Decision 1.** Planner-only, on `pilot_v3b`'s own world seeds: forbidding
reverse steps, for the narrow body or for every body, leaves every method's
plannability unchanged (route-first 1, geometry 18, feasibility 18, treatment
17 of 18), moves one treatment site, and keeps the manipulation (treatment vs
geometry separates in 11 of 12 manipulated layouts, all 6 neutral layouts
agree). In Gazebo, on the seven pilot treatment trials with the failure
signature (`studies/engineering/forward_only/`), the largest narrow in-place
turn fell from 2.26--3.02 rad in six trials to at most 1.37 rad in all seven,
with no contact and every motion qualification passing, against criteria
declared before the run.

**Decision 2, planning side** (`studies/engineering/adr0014_planner_sweep.json`).
With forward-only and a 0.03 m drive-obstacle margin, every method plans in
every layout it planned before, in `pilot_v3b` and both generator-3 Gate 0
designs; treatment-vs-geometry separation stays 11 of 12 manipulated
`pilot_v3b` layouts with all 6 neutral layouts agreeing; and only
`geometry_coupled`'s site moves (8 of 18 `pilot_v3b`, 5 of 5 round-trip, 2 of 3
fault-matrix layouts), away from the shelf. The treatment still plans in the
fault matrix's layouts 00, 01, 02, so its frozen selection rule is unchanged.
A 0.05 m margin, measured earlier, leaves only 4 of 12 manipulated layouts
plannable for any coupled method, so 0.03 m is the ceiling.

**Decisions 2 and 3, executed** (`studies/engineering/drive_margin/`). The
16 minimum-clearance `geometry_coupled` trials: with both, no RPP abort, where
the baseline at `main` aborted in both `combined_constraints-02` trials. The
same 8 layouts for the two methods that sweep their transitions (32 trials):
no abort, no contact, every motion qualification passing, against criteria
declared before the run.

## Consequences

- Plans change (routes, not the manipulation, for item 1), so the manipulation
  check of `pilot_v3b` is recomputed and recommitted before any trial, and the
  generator-3 round trip and fault matrix are re-collected at the fixing
  commit; the fault matrix's layout scan is re-run under its frozen rule.
- `pilot_v3b`'s seeds and layouts do not change.
