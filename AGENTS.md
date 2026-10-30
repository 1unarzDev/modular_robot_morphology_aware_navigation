## Current resumption point

**Pre-data design decisions taken 2026-10-28** under the author's standing
delegation (the author asked the agent to self-direct and decide as they
would; revisit any of these only while pilot evidence is still 0):
smallest effect of interest 0.18; localization loss defined by evaluator truth
and every robot outcome named by its earliest evaluator cause (analysis plan,
"Outcome definitions"); ADR 0010 accepted as A with a per-trial site-placement
measurement; ADR 0011 widens the layout population (generator 2, design kinds
`pilot_v2` / `confirmatory_v2`), because generator 1 allows only 81 near-identical
layouts per family and the frozen 36 already hold 2 duplicates.
**Gate 1 is satisfied (2026-10-30).** The generator-2 pilot (144/144) showed the
study could not detect an effect: the treatment could not execute its
missions. Fixed since, all method-agnostic: ADR 0012 keepout mask, ADR 0013
(0.10 m turning clearance + layout generator 3), start-state repair and
replan budget (`1c1a8c0`), AMCL sensor model (`36953a9`), 0.08 m
placement-robust station prediction (`43255f7`), 4x finer localization map
(`9371593`), and RPP's narrow rotation no longer ratcheting on odometry
(`971affb`; the narrow odometry over-reports ~0.2 rad/s, still unexplained).

**Resume here (2026-10-31):** read the generator-3 treatment round trip left
running overnight at `971affb` -- `results/run_roundtrip_v3e.launch.log` and
`results/qualification/roundtrip_v3e_raw` / `roundtrip_v3e_audit.json` in the
WSL clone (7 of 7 completed when the session ended; it runs to 20 unattended). If it passes, run `pilot_v3`
(`studies/pilot_v3/design.json`, planner check committed) with
`results/qualification/start_campaign.sh <commit> run_pilot` after pointing
run_pilot.sh at the pilot_v3 design; then `pilot_nuisance --power`, Gate 0 on
generator 3, and the `confirmatory_v3` freeze. If it fails, diagnose the
records the same way the earlier v3 runs were (obstacle at first contact,
localization against truth, commanded vs odometry rates). **The frozen clock is root-caused and
prevented** (2026-10-29): gz-transport dropped Gazebo's publishers from the
bridge after 3 s of discovery silence; `GZ_IP=127.0.0.1` in
`simulation.launch.py` took it from 3 of 13 loaded trials to 0 of 14, and a
mission whose clock jumps is recorded as infrastructure (`e5a0401`). External
`gz topic` probes must export the same `GZ_IP`. The planner is ~3x faster with
identical plans (`976e9fc`), so a manipulation-check layout now costs ~1 min.

**Gate 1 was in progress (2026-10-27; satisfied 2026-10-30, above).** `gate1_audit` exists and passes every
telemetry check on the 62 Gate 0 records; records now also carry the container
image and stack process deaths. **Every campaign from here must pass the image
in:** `docker exec -e MORPHOLOGY_CONTAINER_IMAGE=$(docker inspect -f
'{{.Image}}' morphology_navigation_dev) ...`, or `gate1_audit` flags the record.
Owed: controlled collision, localization-loss and timeout records, plus an
author decision on how localization loss is defined (see
`docs/current_status.md`, "Gate 1").

**Gate 0 is satisfied on the repaired model (2026-10-27).** The 42-case fault
matrix passed 42/42 at `d3c2052`, re-executed unchanged (design hash
`de7a6232...`): 14/14 in each of `reconfiguration_workspace-00`, `-03` and `-07`,
all seven fault types twice per layout, every injection fired, no failed checks,
distinct fault seeds. Audit `studies/gate0/fault_matrix_campaign_r4_audit.json`,
campaign digest `8b5d59c7...`, records retained and restore-verified at
`studies/gate0/records/fault_matrix_campaign_r4_raw.tar.xz`. With item 1 (10/10)
and the round trip (20/20), every Gate 0 item now has a post-ADR-0007 audit.
Three cases needed one pre-mission retry each (0 s simulated): one readiness
timeout and two frozen clocks.

**Resume here.** Engineering first: root-cause the frozen simulation clock
(below) before the pilot, since it now recurs at about 3--5% of trials under host
load; `results/qualification/freeze_capture.sh` in the WSL clone captures
simulator thread state when it happens, and it did not recur after being armed.
Then the post-Gate 0 roadmap, which is blocked on three author decisions listed
in `docs/current_status.md`: the smallest effect of interest (0.18 proposed, with
a power preview implying 60--96 layouts), ADR 0010 with a per-trial placement
measurement, and placing the ADR 0006 station and re-freezing the design.

**The 20-run round-trip gate passes on the repaired model, 20 of 20
(2026-10-27).** Collected at `54883d6` with the design unchanged (hash
`9c74a267...`), audit `studies/gate0/roundtrip_20_isolated_audit.json`, campaign
digest `8dba20d1...`, records retained and restore-verified at
`studies/gate0/records/roundtrip_20_isolated_raw.tar.xz`. Every trial made both
transitions with zero failures, passed both motion qualifications including pod
rigidity, and ended `READY` with zero collisions. Worst signed-motion margin is
+0.0301 m forward travel. One trial (r10) needed one retry, and its failed attempt
elapsed 0.000 s simulated, so no robot behaviour was re-run. Zero transport
errors in all twenty.

**Resume here: the 42-case fault matrix**, re-executed unchanged, at one commit,
on an otherwise idle host. Then Gate 1/2 follow-ons and the author decisions
listed in `docs/current_status.md`.

**Open harness item: the simulation clock can freeze at mission start.** Twice
(r15 at `5b00fc9`, r10 at `54883d6`) the observer measured 0.000 s simulated over
the 600 s watchdog after the goal was accepted, and the controller never reached
a first leg that normally takes about 6 s. Both were under 7--9.5 load average
from non-study host applications. It is handled correctly -- recorded as
infrastructure, retried, no robot outcome hidden -- but costs 10 minutes each and
is not root-caused.

**The observer now runs in its own process; the fresh 20 is owed again
(2026-10-27).** The 20 at `5b00fc9` confirmed the maps-aware cleanup -- zero
transport errors in all 16 terminal trials -- and was then stopped at 16 when r16
reached its goal and never terminated. The spin thread blocked in a native rclpy
call that holds the GIL (`rcl_action_client_fini` from inside `is_ready`), so the
in-process 600 s backstop, which needs the GIL, stood 489 s overdue: the
`0e71dd3` pump bounds a wedge in Python, not one holding the GIL.
`mission_batch` now runs each trial's observer in a spawned child that the parent
kills after the watchdog plus 120 s, recording `infrastructure_failure`. The 16
are **not a result, not scored, not audited**; r14 and r15 each needed one
legitimate pre-mission infrastructure retry under heavy host load. See
`studies/engineering/roundtrip_observer_gil_wedge.json`. **Resume here: a fresh
20 at the isolation commit into a new raw directory**, with nothing else running
on the host. The entries below are superseded where they name a commit to
resume at.

**The 0531a0f transport fix itself broke the transport; it is repaired and the
fresh 20 is owed again (2026-10-27).** The interrupted run was resumed at
`0531a0f` and stopped deliberately at 5 of 20 (all five completed, none retried;
**not a result, not scored, not audited**). Its per-trial cleanup judged
segments unused from `/proc/*/fd` alone, but Fast-DDS maps a port and closes the
descriptor, so the runner's own live ports were unlinked after every trial and
each later stack failed to open them: 0, 44, 44, 88, 132 `Failed init_port`
lines in r0--r4, against 0 in 19 of 20 trials at `0e71dd3`. Because an unfinished
trial with any transport error is reclassified and retried, a genuine robot
failure would have been re-run, so the set could not back the gate. The cleanup
now also reads `/proc/*/maps`, with a regression test that fails without it.
See `studies/engineering/roundtrip_shm_cleanup_cleared_live_segments.json`.
**Resume here: a fresh 20 at the fix commit into a new raw directory**; the
`roundtrip_20_shmfix_raw` set stays as collected. In that 20, count
`transport_failed_ports` per trial: it should be empty almost everywhere, and a
trial that is both unfinished and transport-flagged must be read by hand before
its retry is accepted.

The entry below is superseded where it says to resume `roundtrip_20_shmfix_raw`.

**The 20-run round trip now runs to completion and fails one of twenty
(2026-09-30).** At `0e71dd3`: 20 of 20 terminal records, 19 completed, audit
`studies/gate0/roundtrip_20_watchdogfix_audit.json`, campaign digest
`38d8fbd1...`. Every signed-motion gate passes with margin over the 38
qualifications, and real-time factor held 0.9598--0.9796 across all twenty.
**19 of 20 is not the gate; do not report it as a pass.**

Two things are settled by it. The unreachable watchdog is **fixed** -- the
campaign terminated instead of stalling as the previous attempt did for 6 h 15 m
-- and `_trim_transition_yaw` is **confirmed** against the mechanism it was
written for: zero of twenty jammed the doorway, where both earlier attempts put
11 of 20 there. The yaw-trim probe of three trials was not evidence; this is.

**The one failure was a DDS transport failure recorded as a robot outcome, and
is now fixed (2026-10-01).** `r17` is the only trial of twenty whose transport
failed: nine Nav2 nodes each failed to open the same shared-memory port,
`fastrtps_port18186`, while all nineteen others logged none. The robot was at
the site within tolerance with 0.237 m of clearance and never moved a pod,
because the reconfiguration request never reached the executor. `mission_batch`
now records that as `infrastructure_failure` and retries it, and per-trial
teardown clears segments no live process holds open. See
`studies/engineering/roundtrip_transport_failure_misclassified.json`. **The gate
is owed a fresh 20 at the fix commit**; the previous campaign stands as
collected and is not re-scored.

**Resume here.** That fresh 20 was started at `0531a0f` and stopped by an
operator shutdown after one trial, so it is **not a result and not a failure**:
`results/qualification/roundtrip_20_shmfix_raw` holds one terminal record,
`combined_constraints-00-r0-feasibility_coupled`, completed at 157.12 s
simulated. `mission_batch` resumes an interrupted batch and skips terminal
records, so either re-run the same command to finish the remaining 19 -- valid
only while HEAD is still `0531a0f`, since a campaign must run at one commit --
or delete that directory and start the twenty over. Do not score the partial
set, and do not audit it.

What the completed 20 will and will not show: it tests whether the gate passes
with the transport path fixed, but it cannot confirm the transport fix, which
needs a recurrence to exercise. A clean 20 with no retries leaves that fix
covered by `tests/test_dds_transport_failures.py` only. Say so either way.

After the round trip passes, the 42-case fault matrix is owed, in the roadmap's
order. ADR 0010 is proposed and is an author decision: whether to report
`site_position_tolerance` as nominal rather than physical. Nothing in the
repository is waiting on code for it.

**ADR 0005 is accepted as A (2026-09-30, author decision).** Campaigns that back
a claim are retained as verified `.tar.xz` archives beside their audits;
`retain_campaign_records` archives and restore-verifies in one step. The
confirmatory set is no longer blocked on retention.

**The fault matrix is still owed**, after the round trip, in the roadmap's order.

Earlier state, still accurate except where the entry above supersedes it:

**ADR 0007 is adopted (2026-09-26): the wheel friction axes are attached to the
wheel.** `fdir1 1 0 0` on both wheel collisions of `articulated_pod`, with the
declared `mu 1.2` / `mu2 0.08` unchanged. Without it the friction principal axes
were world-fixed, so traction depended on the robot's heading: rotation
authority ran 73% of command near 0 and 180 degrees, 45% by 45 degrees and
**0% at +/-90**, and straight travel lost 21% at 90 degrees entirely in the
acceleration ramp. The measurements are
`studies/engineering/wheel_friction_intervention.json` (confirmed by
intervention, protocol declared first) and
`studies/engineering/heading_traction_curve.json` (the full curve, heading-local
rather than from stage totals). A general test now fails any collision declaring
`mu != mu2` without `fdir1`, and the unloaded `drive_pod` model -- which the
first diagnosis wrongly blamed -- is deleted.

**This un-satisfies Gate 0, by repair rather than regression.** Every motion
figure on record was collected with world-fixed axes.

- Item 1 is **re-collected, 10/10** at `b359903`
  (`studies/gate0/gate0_item1_fdir1_audit.json`), and now includes four
  heading-varied cases: assembly rotation 1.0196--1.0254 rad and straight travel
  0.7381--0.7429 m across 0, 45, +90 and -90 degrees, against 0.8522 rad /
  0.7397 m and 0.0000 rad / 0.5830 m before.
- The 20-run round-trip audit and the 42/42 fault matrix are **superseded and
  not re-scored**; both need re-collecting, in that order. They would have
  needed it for the ADR 0006 work anyway.

**ADR 0009 is adopted**: site alignment floors its pure in-place turn at
0.30 rad/s (never while translating) and abandons alignment after 5 simulated
seconds without 0.02 rad of yaw, instead of holding a dead command for the whole
30 s window.

**ADR 0008 is adopted, and half of it was withdrawn on measurement.**
`post_transition_verification.margin_m` 0.0 -> 0.07 m, which is the ceiling less
0.01 m of headroom (the sweep reaches 0.92 m inside a 1.0 m disk and grows
one-for-one). `site_position_tolerance` **stays at 0.030 m**: raising it to 0.07 m
failed the 20-run round trip 11 of 20, every failure jamming the 1.76 m narrow
body's nose in the 0.70 m doorway, whose 0.66 m footprint leaves 0.02 m of slack
per side. The failed campaign is retained at
`studies/gate0/roundtrip_20_fdir1_attempt1_failed_audit.json`. The doorway demands
a placement tighter than the estimator can resolve, so the original defect is a
platform limitation, not a tuning knob; the options are wider doorways, a
pre-doorway re-centring maneuver, or reporting the tolerance as nominal. Do not
raise it again without changing the scenario geometry.

**The 20-run round trip is the open work.** Two attempts on the repaired model
both failed 11 of 20, with different replicates failing each time, so it is
marginal rather than a threshold. Measured cause: the transition itself injects
+0.011..+0.134 rad of core yaw (alignment leaves at most 0.030 in every run), the
drive that follows amplifies it, and a 0.70 m doorway against a 0.66 m footprint
gives 0.02 m of slack per side. `_trim_transition_yaw` now trims the core yaw to
0.02 rad after a commit; a 5-trial probe was stopped after 3, all completing,
including two replicates that had failed without it
(`studies/engineering/roundtrip_yaw_trim_probe.json`, three trials and not
evidence). **Execute the full 20 and audit it before claiming anything.** Both
failed attempts are retained and not re-scored. When running a campaign, write the
audit under `results/` and copy it into `studies/` afterwards: an untracked file in
`studies/gate0/` makes the worktree dirty and `mission_batch` then refuses to run
at all.

**A third attempt ran on 2026-09-27 and was stopped by a harness fault at run 10,
rather than by an outcome. The blocker is now the harness, not the robot.**
Replicates r0--r8 all completed -- two transitions each, zero collisions, 18 of 18
motion qualifications passed including pod rigidity -- and then r9 wedged its
mission observer and sat there for 6 h 15 m. The trial's 600 s wall watchdog is
evaluated *after* `executor.spin_once()` in the same loop body, and spin_once never
returned, so the backstop is unreachable: the break condition was continuously true
for 22,152 s while the line below it never ran (`mission_trial.py:571-577`; the
executor's timeout timer is STEADY_TIME, so this is not a simulated-clock bug).
Gazebo, the bridge and the Nav2 nodes stayed healthy and reached simulated
20,099.642 s while the observer's clock stayed frozen at 163.512 s. The set is
retained whole and **unscored** at
`studies/gate0/roundtrip_20_yawtrim_partial_quarantined_audit.json`; the fault, with
the py-spy locals and the digests, is
`studies/engineering/roundtrip_watchdog_unreachable.json`. **Both are done**: the
fix and its wedging regression test are at `0e71dd3` and the fresh 20 ran at that
commit. A wedged observer must record as `infrastructure_failure`, which
`mission_batch` already retries, and never as a robot outcome. Nine of nine
completing is consistent with `_trim_transition_yaw` working and is not the
twenty-run gate; do not report it as a result.

**A harness fault was found and fixed while re-collecting.** No scratch harness
here ever killed `parameter_bridge`, so bridges accumulated across cases while
the same teardown deleted `/dev/shm/fastrtps_*` underneath the live ones: Gazebo
publishes `/clock` normally while no ROS participant can discover anything, and
a tool times out at readiness. Kill bridges and count them in the cleanliness
guard. The first item-1 attempt is quarantined whole for this reason.

Earlier context, kept because the reasoning still matters. Its "Gate 0 is
satisfied" claim and its round-trip and fault-matrix statements describe the
pre-repair model and are superseded by the entry above:

Gate 0 was satisfied on 2026-09-21: every roadmap item had a committed audit in
`studies/gate0/`. The older records
behind every previous Gate 0 claim are absent from this machine -- `results/`
is gitignored and no archive exists -- so the gate's required artifact cannot
be produced from them. They needed re-collecting regardless, for the reason in
the next paragraph. Treat the Gate 0 figures in `docs/current_status.md` as
unverifiable prose until a campaign replaces them.

Transition feasibility now sweeps the post-transition verification maneuver
(ADR 0003), which closed the failure the workshop diagnostic exposed; the nine
workshop missions were re-run end to end and both previously failing Blocked-A
missions now complete.

That intermittent post-transition motion gate was an evaluation-harness fault,
not mechanics. Commanded maneuver windows were bounded by the wall clock while
the robot moves in simulated time, so travel scaled with the real-time factor.
Windows now advance on the simulated clock, and travel is confirmed invariant
across a 0.54--0.98 real-time-factor span. Every signed-motion figure recorded
before that fix scales with whatever host load its run happened to meet, which
is why the lost campaigns could not have supplied a margin. `morphology
qualify_roundtrip_batch audit` now reports `motion_margins` and
`real_time_factors`, so the replacement campaign yields the margin directly.

The 20-run round-trip gate passed, Gate 2's manipulation check is closed, and
the multi-layout fault matrix passed 42/42 on its third execution
(`studies/gate0/fault_matrix_campaign_r3_audit.json`). Getting there adopted
ADR 0004 (raised obstacles constrain driving; the shelf moved off the route),
and fixed pod_4's final alignment, executor waits on the wall clock, the
auditor's blindness to unfired faults, and transforming away from the planned
site pose. Detached-pod and compact-assembly qualification (roadmap item 1)
was re-collected, 6/6 passing (`studies/gate0/gate0_item1_audit.json`). Next
is the post-Gate 0 roadmap: location-dependent perceived obstacles and
observability, the disjoint pilot, then the confirmatory freeze. See
`docs/current_status.md` "Resume here", and its retention
section for the storage decision the confirmatory set forces: records run
3.5--5.8 MB each, so 432 trials is about 1.9 GB and plain git tracking is not
viable.

**Two author decisions now block progress; do not resolve either in code.**
ADR 0005 proposes the record-retention mechanism the confirmatory set forces.
ADR 0006 concerns the observability manipulation: both sensing-contrast
families declare poor-observability regions that nothing in the world can
produce, because the connector sensors are frustum-only logical cameras and
covariance is a function of range alone. Wiring the planner's existing per-site
`sensing_provider` to the declared regions would close roadmap item 3 as
literally written while leaving the sensing contrast with no outcome mechanism,
so it is not a shortcut worth taking. Giving it a physical cause by occlusion
was chosen and then measured to be impossible: the connector cameras sit at the
core origin and the pods move radially outward, so every sight-line-blocking
placement is inside the robot's own footprint. ADR 0006 is now **accepted as
A1** and partly built: observability is established by an external workspace
fiducial station, occluded by an *elevated* screen that clears the robot
entirely. The geometry, the planner predictor, the simulator station node and
the sensing gate are in; the station is not yet placed in any scenario, so
nothing has changed behaviourally. Keep every step backwards compatible — a
world declaring no station must behave exactly as before, which is what leaves
the round-trip and fault-matrix gates untouched.

Placement is in. A site is rejected when any moved pod is shadowed, so the
rejected set is the shadow dilated by the pods' reach and the shadow is the
declared region eroded by it; because the pods sit at four discrete offsets, a
contiguous rejected band needs a declared region of about 2.12 m, so
`combined_constraints` was widened from 0.60 m to 2.10 m (author decision,
2026-09-22). Gate 2 is regenerated: 9/9 separating, 0 unplanned in all four
cells, all six neutral controls agreeing, and 0 of 24 geometry and feasibility
sites changed. The container builds and the runtime path works: a smoke run shows
`geometry_coupled` choosing a poorly observed site and being refused by the
docking gate (`connector_not_visible`, `sources=connector_camera,
wheel_odometry`), which is the outcome mechanism ADR 0006 existed to create.

**Open blocker: site alignment deadlocks before the transition.**
`sensing_feasibility_coupled` avoided the shadowed site, planned one about
0.20 m off the route line at 45 degrees, and failed `_align_to_site` three times
with zero transition attempts. The mechanism was corrected on 2026-09-25 from
the records, and it is not the loose Nav2 handover the first write-up blamed:
`site_alignment_command` gates translation behind `|bearing| <= 0.15 rad` and
commands `min(0.6, 1.5*|bearing|)` rad/s of in-place rotation until then, and
the assembled skid-steer cannot break away from rest at the low end of that
range. Two of the three 29.5 s windows held a constant 0.271 rad/s and turned
the body 0.000 rad, wheels spinning at 0.91 of 0.99 rad/s commanded at
0.08 N*m, against 0.260 N*m while the same run was turning successfully at a
*lower* command of 0.166 rad/s. Suspension was unchanged, so the wheels were on
the ground; they spin nearly free and do no work, and grip is never recovered.
**Retracted 2026-09-26: there is no commanded-rate floor.** The 2026-09-25
handoff inferred breakaway in (0.271, 0.350] rad/s and a bearing dead zone up to
0.23 rad. A direct sweep (`qualify_assembly_motion --yaw-rate`, new) rotates the
assembly from rest at every rate from 0.20 to 0.35 rad/s at 58--75% of command
in the fixed world Gate 0 used, 0.27 rad/s included; see
`studies/engineering/rotation_authority_sweep.json`, whose 0.35 control
reproduces the committed Gate 0 figures. The companion claim that no gate covers
the navigator's regime is retracted with it. **The cause is the heading of the robot in the
world.** Imposing poses in the smoke world at 0.271 rad/s: rotation reaches 65%
of command at yaw -0.03, 58% at -0.8, **exactly 0% at both +1.571 and -1.571**,
and 65% again at +3.142 -- a 180-degree period. The original run froze at yaw
-1.577 with no teleport. Position, teleporting, the world, the commanded rate,
attitude, suspension and actuator stall are all eliminated by controls. The
wheels declare anisotropic friction (`mu 1.2`, `mu2 0.08`,
`articulated_pod/model.sdf.in:19-20`; the earlier citation of
`drive_pod/model.sdf` named a model nothing loads) and **`fdir1` is never
specified anywhere**, so the
friction axes are not attached to the wheel and traction depends on world
heading; `kinematics.py` is innocent, being body-frame only. **Confirmed by
intervention 2026-09-26** (protocol declared first at `e2bc940`; patch applied
to the built tree inside the container only, restored and verified, source
template untouched, so no figure moved): both `fdir1 1 0 0` and isotropic
friction remove the heading dependence completely -- the same rotation and the
same straight travel to four decimals at +1.571, -1.571 and 0 -- while the
unmodified tree still gives 0.000 rad at +90 in the same session. `fdir1` keeps
the declared anisotropy and rotates at 74% of command at every heading, better
than the most favourable heading ever produced; isotropy gives 46%, because it
raises lateral scrub resistance. The engine does read `fdir1`. Straight travel
is heading-dependent too, 21.2% short at +/-90, which would take the round-trip
gate's forward margin from +0.0317 m to about +0.0091 m. Numbers with digests:
`studies/engineering/wheel_friction_intervention.json`. The full curve is then in
`studies/engineering/heading_traction_curve.json`: rotation authority is
symmetric about 90 degrees with a 180-degree period and falls off across most of
the circle rather than notching -- 73% of command near 0 and 180, 66% by 30, 45%
by 45, 26% by 65, 4--9% within 10 degrees of 90 -- so the gate's yaw floor
projects to fail for about a third of all headings. Stage totals cannot measure
this (a 4 s stage integrates authority over the arc it sweeps); the figures come
from local rate against instantaneous heading in the recorded traces. Straight
travel loses its ground in the ramp and not at speed, so the gate's forward
floor cannot be assessed by scaling, and the detached pod is affected in travel
but not in yaw. **Which repair to adopt is still an author decision** because it moves Gate 0 item 1, the round-trip
audit and the fault matrix, all collected near the most favourable heading while
confirmatory layouts route in all directions; ADR 0007 is `proposed` and holds
the options and a re-collection order. Independent of the cause, the law turning a failed rotation into
a hard stop -- it refuses to translate while `|bearing| > 0.15 rad`, and its
bearing branch has no angular floor while its yaw branch has 0.15 rad/s -- is a
robustness defect; `reconfiguration_executor` already applies
`min_pod_angular = 0.30` to pod in-place turns (`node.py:408-417`). Separately, `site_position_tolerance` (0.030 m) sits below the
localization error the controller must servo against (0.0296 m at the one
successful alignment, up to 0.1021 m), so the 1.3 mm margin recorded earlier is
not a physical margin. Reduced trace with digests:
`studies/engineering/adr0006_alignment_diagnosis.json`. Both are author
decisions, about rotation authority (ADR 0007) and about the tolerance; do not
resolve either as a side effect of a tuning change, and see
`docs/current_status.md` for the options. The fault matrix is still not
re-qualified. ADR 0005 is now accepted as A; see the entry at the top.
