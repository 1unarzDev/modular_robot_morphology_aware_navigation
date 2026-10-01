# ADR 0010: The doorway slack and the nominal site tolerance

- Status: **proposed** (awaiting author decision)
- Date: 2026-09-30

## Context

ADR 0008 raised `post_transition_verification.margin_m` to 0.07 m, measured that
`site_position_tolerance` could not be raised, and handed off three options as
"all larger than this ADR": widen the confirmatory doorways, add a pre-doorway
re-centring maneuver that servos on something better than AMCL, or accept and
report the tolerance as nominal rather than physical. This ADR settles that
handoff.

**New evidence changes what those options are worth.** The 20-run round trip was
re-collected at `0e71dd3` on 2026-09-30 and completed 19 of 20 with **zero
doorway jams** (`studies/gate0/roundtrip_20_watchdogfix_audit.json`). Both
earlier attempts on the repaired model failed 11 of 20, and every one of those 22
failures was the same geometric event: the narrow body's nose jamming in the
0.70 m doorway. After `_trim_transition_yaw`, that event does not occur. Where
the failed campaign logged Nav2 "Failed to make progress" 33 times, the one
failure in the new campaign logs it not at all.

So the doorway is no longer producing failures, and the measured cause of those
failures was never the doorway's width -- it was core yaw injected by the
transition and amplified by the drive that followed. The geometry was the
constraint that turned a yaw error into a collision; removing the yaw error
removed the collisions at unchanged geometry.

What that evidence does **not** settle is the original defect ADR 0008 recorded.
`site_position_tolerance` is 0.030 m and the AMCL estimate the controller servos
had 0.0221--0.0647 m of error on one run and 0.0516--0.1021 m on another. A
tolerance below the estimator's own error means "reached the site" asserts a
placement precision the estimate cannot resolve, whether or not the traversal
that follows succeeds. That is a reporting problem now, not a traversal problem.

## Options

- **A. Report the tolerance as nominal, not physical (recommended).** Record that
  site arrival is asserted against an estimate whose error reaches 0.1021 m, so
  0.030 m is a control target rather than an achieved-placement bound, and that
  `margin_m` 0.07 m is what covers placement error -- partially, since error
  beyond 0.04 m stays unmodelled. Changes no geometry, no tolerance and no
  scenario; costs a qualification on every figure derived from site placement.
- **B. Widen the confirmatory doorways.** Now **contra-indicated**. No current
  failure motivates it; it changes scenario geometry the confirmatory design
  depends on, which must be frozen before data exists; and ADR 0008 measured that
  the larger clearance disks a wider sweep would permit break the golden doorway
  fixture outright at `swept_radius` 1.15 and stop `combined_constraints`
  separating from 0.13. Adopting it after a campaign that passed the doorway
  would also be a scenario change made in response to an outcome.
- **C. Add a pre-doorway re-centring maneuver.** Unmotivated by current evidence.
  The measured cause is already addressed at the point the error is introduced,
  which is strictly earlier and cheaper than correcting for it 1.1 m later. A
  second corrective maneuver would add a mechanism with no failure to justify it
  and new motion to qualify.
- **D. Defer until `r17` is root-caused.** The single remaining round-trip failure
  is not doorway-related -- no pod moved, the robot was inside tolerance at the
  site, clearance was 0.237 m -- so this ADR is not on its critical path. But a
  decision taken before that cause is known could be revised by it.

## Decision (proposed)

Adopt A, and record B as contra-indicated by the 2026-09-30 campaign rather than
merely unchosen, so it is not revisited as an option without new evidence. C stays
available if a future campaign reproduces placement-driven doorway contact at
unchanged geometry.

A is not a fix and should not be written up as one. It is the honest reading of a
platform limitation that ADR 0008 already established and that this campaign did
not remove: the estimator cannot resolve the placement the scenario asks for, and
what changed is that the robot no longer needs it to.

## Consequences

Anything reporting site placement carries the qualification that the figure is
nominal. The uncovered estimator error above 0.04 m remains unmodelled, so a
transformation can still execute about 0.13 m from the pose the planner checked in
the worst case observed. Both are consequences of the geometry, and ADR 0008's
note stands: transition workspaces with enough slack that geometry stops being the
binding constraint is a scenario-design change, not a tuning change.

The round-trip gate is not passed, and this ADR does not pass it. `r17` is owed a
root cause first.
