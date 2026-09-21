# ADR 0003: Transition feasibility covers every motion performed at the site

- Status: accepted
- Date: 2026-09-19

## Context

ADR 0002 admits a reconfiguration edge after checking the module relocation
that connects two morphologies. The workshop diagnostic showed that this is not
the whole physical commitment a site makes. At a site the check accepted, the
executor committed `narrow_tandem`, and the automatic post-transition motion
check then yawed the 1.6 m assembled body in place and drove pod 0 into an
obstruction that no relocation path touched. Both configurations fit, the
relocation was clear, and the site was still unusable.

Reconfiguring at a site implies more than the transformation: the robot also
performs the assembled motion that qualifies the new morphology there. That
motion was executed but never planned against.

## Decision

The post-transition verification maneuver is declared once, as
`post_transition_verification` in the morphology catalog, and both sides
consume that single declaration. `morphology_planner` sweeps the target
morphology rigidly through the commanded twists and rejects an edge whose sweep
collides, with reason `<module>:post_transition_collision`.
`modular_robot_bringup.qualification.VERIFICATION_SEQUENCE` executes it, and a
test binds the two so the planner cannot accept a site against a maneuver the
robot does not perform.

The check belongs to the feasibility-coupled methods only. `geometry_coupled`
remains a geometry-only ablation.

## Consequences

Site selection now accounts for the full motion the site must host, and on the
frozen workshop scenarios it rejects exactly the site whose physical failure
motivated the change, while leaving the unobstructed control untouched.

The general rule this establishes -- a reconfiguration site must admit every
motion performed there, not only the transformation -- is not yet fully
discharged. Post-transition navigation away from the site is still governed by
the ordinary traversal checks, which read only the occupancy grid, so declared
3D obstacles are invisible to traversal edges; sampling the assembled body
along planned routes found no current layout that routes through one. The
sweep also models the commanded nominal motion: `margin_m` exists for
execution tolerance and is 0.0, while recorded gates overshoot the commanded
yaw by 41%, worth 0.145 m of unswept arc at the farthest corner. Current
decisions are robust to that overshoot, but the tolerance should be derived
from the Gate 0 records, and a linear box inflation is an imperfect model of
an angular error.

Adding a feasibility term changes which sites the coupled methods prefer, which
can invalidate a Gate 2 golden layout that was tuned to separate an ablation
under the previous model. This happened to `combined_constraints`, which was
re-selected to a world seed that separates under both the old and the
corrected model, against a recorded seed sweep, with no pilot or confirmatory
data in existence. Such a re-selection must never be made in response to an
outcome.
