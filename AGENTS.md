# Repository handoff

Read [README.md](README.md), [docs/current_status.md](docs/current_status.md),
and [docs/research/conference_roadmap.md](docs/research/conference_roadmap.md)
before changing the platform or collecting data.

## Non-negotiable study rules

- Confirmatory morphologies are `compact_diff` and `narrow_tandem` only.
- Compare `route_first_adaptation`, `geometry_coupled`,
  `feasibility_coupled`, and `sensing_feasibility_coupled` through the same
  execution and sensing stack.
- Simulator ground truth is evaluator-only. Do not use it for localization,
  planning, docking, or recovery.
- Any partial or inconsistent topology must enter `RECOVERY_REQUIRED`; keep
  assembled drive inhibited until observed topology is reconciled.
- `results/debug` contains engineering runs. It is never pilot or confirmatory
  evidence.
- Do not collect pilot data until the platform and measurement gates in the
  roadmap pass. Confirmatory evidence is currently 0/432.
- Do not claim the first morphology-aware navigation stack or first autonomous
  ROS reconfiguration system. Cite Pankert et al. (2022), not Gawel, for the
  simultaneous navigation/reconfiguration MPC work. Treat announced 2026
  workshop talks as unpublished.

## Working conventions

- Keep the canonical morphology and transition geometry in
  `src/modular_robot_description/config/morphologies.yaml`.
- Put ROS interfaces in `modular_robot_msgs`, pure planning logic in
  `morphology_planner`, topology ownership in `morphology_manager`, physical
  transition execution in `reconfiguration_executor`, simulation mechanics in
  `modular_robot_sim`/`modular_robot_gz_plugins`, integration in
  `modular_robot_bringup`, and experiment logic in `modular_robot_benchmarks`.
- Update `docs/current_status.md` when an evidence gate changes. Record durable
  architectural choices as an ADR; rely on git history for chronological work
  logs.
- Preserve failed runs and structured failure reasons. Never change exclusions,
  scenarios, or sample size in response to confirmatory outcomes.
- Run `python3 -m pytest -q`, the container build, and `git diff --check` before
  committing a platform milestone. Kill leftover Gazebo/ROS processes after
  runtime tests.

## Current resumption point

Gate 0 is satisfied (2026-09-21): every roadmap item has a committed audit in
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

The 20-run round-trip gate passes, Gate 2's manipulation check is closed, and
the multi-layout fault matrix passes 42/42 on its third execution
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
heading; `kinematics.py` is innocent, being body-frame only. Confirmed by
signature, not by intervention: setting `fdir1` or making the friction isotropic
and re-running +/-90 is the decisive test, and it is **an author decision**
because it touches every recorded motion figure -- Gate 0 item 1 is collected at
the spawn heading near zero, the most favourable one, while confirmatory layouts
route in all directions. Independent of the cause, the law turning a failed rotation into
a hard stop -- it refuses to translate while `|bearing| > 0.15 rad`, and its
bearing branch has no angular floor while its yaw branch has 0.15 rad/s -- is a
robustness defect; `reconfiguration_executor` already applies
`min_pod_angular = 0.30` to pod in-place turns (`node.py:408-417`). Separately, `site_position_tolerance` (0.030 m) sits below the
localization error the controller must servo against (0.0296 m at the one
successful alignment, up to 0.1021 m), so the 1.3 mm margin recorded earlier is
not a physical margin. Reduced trace with digests:
`studies/engineering/adr0006_alignment_diagnosis.json`. Both are author
decisions, about rotation authority and about the tolerance; do not resolve
either as a side effect of a tuning change, and see `docs/current_status.md`
for the options. The round-trip and fault-matrix gates are still not
re-qualified. ADR 0005 remains `proposed`.
