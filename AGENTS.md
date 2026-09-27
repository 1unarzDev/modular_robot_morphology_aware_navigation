## Current resumption point

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
`docs/current_status.md` for the options. The round-trip and fault-matrix gates are still not
re-qualified. ADR 0005 remains `proposed`.
