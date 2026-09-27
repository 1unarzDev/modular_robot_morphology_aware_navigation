# ADR 0009: Site alignment fails fast instead of deadlocking

- Status: **accepted**, 2026-09-26
- Date: 2026-09-26

## Context

Before a transition the navigator aligns to the planned site pose, because
feasibility is checked at that pose while the path follower stops anywhere
within 0.12 m at any heading (`yaw_goal_tolerance` 3.14159). That binding is
what took the fault matrix from 38/42 to 42/42.

The law has two defects that are independent of why any particular rotation
fails, and the ADR 0006 station smoke run exposed both. It refuses to translate
while `|bearing| > 0.15 rad` and commands a pure in-place turn until then, so a
rotation that does not happen becomes a hard stop rather than degraded progress:
`body_linear_x` was 0.000 in **every one of 60 recorded samples** across three
29.5 s windows, 88.5 s of commanded rotation that turned the body 0.000 rad,
while the robot sat 0.098 m from a site it had already reached to within 0.12 m.
And the bearing branch has **no angular floor at all** while the final yaw branch
has 0.15 rad/s, so an arbitrarily small rate could be asked to break the
assembly away from rest.

ADR 0007 repairs the cause of that particular stall: rotation authority was zero
at the -1.577 rad heading the mission froze at, and is 74% of command at every
heading with the friction axes attached. That does not repair the law. A floor
would not have rescued the recorded failure either, since 0.271 rad/s already
produced nothing.

## Decision

Both, as robustness rather than as a fix for ADR 0007's defect.

1. **A breakaway floor on the pure-turn branch only.**
   `site_alignment_command` takes `min_bearing_angular`, default 0.30 rad/s,
   matching `min_pod_angular` in `reconfiguration_executor` (`node.py:408-417`).
   It applies only where the command is a pure turn. Once translation is
   authorized the floor is not applied, because flooring a turn while
   translating steers a moving body -- which is what the executor's own comment
   warns against -- and the final yaw branch keeps its 0.15 rad/s floor because
   it trims inside the tolerance rather than breaking away.
2. **A rotation-stall fallback.** `RotationStallWatch` reports a commanded
   in-place rotation that has not produced `site_alignment_rotation_progress_rad`
   (0.02 rad) of yaw in `site_alignment_rotation_stall_s` (5.0 s) of simulated
   time. Progress re-arms it, so a slow turn is not a stall and only an absent
   one is. The navigator then abandons alignment and replans instead of holding
   the command for the rest of the 30 s window. Replanning can succeed where
   waiting cannot: the site is the thing that is wrong.

Both are parameters, so neither is a hidden constant, and the watch is fed
simulated time like every other window in the qualification path.

## Consequences

Alignment now fails in about 5 s instead of 30 where it cannot rotate, and the
log names the commanded rate, the elapsed simulated time and the progress
threshold. `_align_to_site` returning false already sets `execution_failed` and
replans up to `max_replans`, so no caller changes.

This does not by itself make the ADR 0006 sensing contrast executable. The
sensing site was 0.098 m from the estimate's target against a 0.030 m tolerance
that sits below the estimator's own error, which is ADR 0008.

The round-trip and fault-matrix gates run through this law, so both need
re-qualification after this and ADR 0007 together. They already did, from the
ADR 0006 work.
