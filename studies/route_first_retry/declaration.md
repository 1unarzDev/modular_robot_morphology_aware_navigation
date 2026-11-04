# Exploratory check: is the route-first rung weak only because it never retries?

Declared 2026-11-04, before any tuning result was read and before the variant
was run on any layout of the frozen design, under the author's standing
delegation. **Exploratory**: planner only, no hypothesis test, and it changes
nothing in `confirmatory_v3`'s design, methods, analysis or interpretation
rules. `route_first_adaptation` in the confirmatory study stays the frozen
single-route rung.

## Why

`route_first_adaptation` plans 5 of the 60 confirmatory layouts (manipulation
check, before any trial). A reviewer can fairly ask whether a sequential
planner that tries another route after a failed adaptation would close most
of that gap, in which case the rung is a strawman rather than evidence about
sequential planning.

Disclosed: one trace on tuning layout `reconfiguration_workspace-20`, made
while writing the variant, found the shortest route does pass cells where
`compact_to_narrow` is feasible, but the narrow body cannot execute the
route's in-place turn and diagonal jog onto the door line, and no feasible
site lies after that jog. On three tuning layouts (one per family), K = 10
distinct routes at radius 2, weight 1 all failed adaptation. Nothing on the
design layouts was run.

## Variant

`modular_robot_benchmarks.route_first_retry.RouteFirstRetryPlanner` at the
commit that adds this file: the frozen rung's morphology-agnostic spatial
search and fixed-route adaptation, unchanged; after a failed adaptation every
cell within `penalty_radius` (Chebyshev) of that route gains one count, a
spatial edge into a cell costs `1 + penalty_weight x count` times its
geometric cost, and a new route is planned, up to `max_routes`. Transitions
are validated by the treatment's own policy, as for the frozen rung. Heading
bins 8, epsilon 2.5, as in missions.

## Tuning, and the rule that picks the declared configuration

Tuning layouts: superset layouts 20-39 of each family
(`studies/confirmatory_v3/manipulation_check_superset40_*.json`), which are
outside the frozen design. Two configurations, both run in full:

- A: `max_routes` 10, `penalty_radius` 2, `penalty_weight` 1;
- B: `max_routes` 20, `penalty_radius` 4, `penalty_weight` 2.

The declared run uses the configuration that plans more tuning layouts; on a
tie, A (fewer routes). No other configuration is added after either result is
read.

## Sample and procedure

The 60 layouts of `studies/confirmatory_v3/design.json` (20 per family),
planned once each by the declared configuration and by
`sensing_feasibility_coupled`.

## Reported, and nothing else

The number of layouts planned within k routes for k = 1..K; the treatment's
planned count; among layouts both plan, the variant's plan cost relative to
the treatment's and how often it chooses the treatment's transition site; and
the tuning results of both configurations. No layout is excluded, and the
variant is not changed after the design-layout run.

## What it can claim

Whether route replanning, in this form, removes the rung's planning
failures on this population. It cannot show that no sequential planner
would: only that this natural one does or does not.
