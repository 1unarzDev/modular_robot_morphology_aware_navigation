# ADR 0011: A layout population worth generalizing over

- Status: **accepted** (2026-10-28, under the author's delegation of pre-data
  design decisions; confirmatory and pilot evidence are both 0)
- Date: 2026-10-28

## Context

The analysis plan makes the layout the independent sampling unit and estimates
a layout-balanced effect "over the generated layout population". That claim is
only as broad as the generator. Generator 1 (`make_confirmatory_scenario` as it
stood) draws four things per layout:

| Draw | Values |
|---|---|
| interior wall x | 3.0 m +/- 1 cell (3 values) |
| doorway centre y | 1.75 m +/- 1 cell (3 values) |
| staging band end | 3 values, 0.05 m apart |
| staging band half-height | 3 values, 0.05 m apart |

The arena is always 6.0 x 3.5 m, and the start and goal always sit on the door
line, so every route is a straight line through the doorway. A family therefore
has at most 81 distinct non-neutral layouts and 9 distinct neutral ones, and the
largest difference between any two is 0.1 m in the wall and door.

Two consequences, measured on 2026-10-28:

1. **The frozen 36-layout design already contains duplicates**: 34 distinct
   layouts, with one neutral `combined_constraints` layout and one
   `docking_observability` layout each drawn twice. Two identical layouts are
   not two sampling units.
2. **The design cannot grow.** The smallest effect of interest (0.18, decided
   2026-10-28) implies 60--96 layouts in the illustrative power preview, i.e.
   20--32 per family, of which a quarter are neutral controls with only 9
   possible forms. The generator also refuses any layout index above 11.

A reviewer reading "held-out layouts" would assume environments that differ;
these differ by 10 cm. That is the claim's weakest point, and it can only be
fixed before pilot data exists.

## Decision

Add generator 2, bound to the design kind (`pilot_v2`, `confirmatory_v2`) so
the design hash covers it, and use it for the pilot and the confirmatory
design. Generator 1 stays byte-identical, so every engineering campaign and its
retained records still name the worlds they ran in.

Generator 2 keeps the structure every manipulation depends on -- one interior
wall with one 0.70 m doorway, a start room the robot must stage in, a goal room
beyond -- and varies what a layout population should:

| Draw | Generator 2 |
|---|---|
| arena | 5.7--6.6 m x 3.2--4.0 m |
| interior wall x | 2.6--3.4 m |
| doorway centre y | anywhere leaving at least 0.7 m of wall either side |
| start y | door line +/- 0.3 m |
| goal y | door line +/- 0.6 m |
| staging band end, half-height | 5 values each |

Start and goal off the door line mean a route must turn to reach the doorway,
so the robot's approach and the transition site's position relative to the
start now vary between layouts. Layout indices are unbounded.

Not adopted: clutter boxes in either room. They would add navigation variety
but no new interaction with the transition decision the study is about, and
each one is another way for a trial to fail for reasons unrelated to the
contrast. Revisit only with a reason the contrast needs.

## Measurement

A screening sample before adoption, not the Gate 2 artifact: 4 layouts per
family drawn by a throwaway `pilot_v2` design (master seed 99991, never run),
every method planned from the scenario's priors exactly as
`check_planner_manipulation` does (heading bins 8, epsilon 2.5).

| Check | Generator 2 |
|---|---|
| sensing vs geometry, non-neutral layouts, all families | 9 of 9 separate |
| sensing vs feasibility, non-neutral layouts, sensing families | 6 of 6 separate |
| neutral controls where the three coupled methods agree | 3 of 3 |
| slowest single plan (after `976e9fc`) | 26.5 s |
| `route_first_adaptation` plans at all | 1 of 12 |

The contrasts survive the wider population with the same pattern generator 1
showed (9/9 and 3/3 on the frozen 36). One thing changes: under generator 1,
`route_first_adaptation` failed with `NoPathError` in every non-neutral layout
but planned in the neutral ones; under generator 2 it also fails in two of the
three neutral controls, because a start off the door line gives it a fixed
spatial route that no feasible transition can be adapted onto. It is in
neither confirmatory contrast and its planning failures already score as
mission failures (decided 2026-10-27), so this changes no estimand. It does
sharpen a fair reviewer question -- whether that baseline is a straw man --
and the paper must report its planning-failure rate rather than leave it in
the completion numbers.

Gate 2's neutral-control requirement is therefore read for the three coupled
methods, which are the ablation ladder the contrasts compare; the full report
on the frozen generator-2 design is still owed before the confirmatory freeze.

## Consequences

- `studies/confirmatory/design.json` (kind `confirmatory`, generator 1) is
  superseded and never runs; the confirmatory design is re-frozen as
  `confirmatory_v2` after the pilot's power grid fixes the layout count.
- The Gate 2 report must be regenerated on the generator 2 design before the
  confirmatory freeze.
- Gate 0 was qualified on generator 1 layouts. The mechanics it qualifies do
  not depend on the arena, but a turning approach to the doorway is new
  behaviour, so the pilot is also the first evidence of the platform on it; a
  robot failure there is an outcome, never a reason to narrow the generator.
