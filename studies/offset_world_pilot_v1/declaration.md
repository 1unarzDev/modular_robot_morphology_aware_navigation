# offset_world_pilot_v1: declaration

Written and committed before any trial of this design exists. Design
`design.json` (hash `0bd851b8f37e621d803514ba342934664131c741af2930cdf7ba967e6e33d6ab`,
208 trials); stated predictions `predictions.json` (cell by cell, from geometry);
smoke subset `smoke_trial_ids.json` (12 trials).

**This is a pilot. It is never pooled with `confirmatory_v3` or any other study
data, it licenses no confirmatory claim, and its records are kept apart
(`results/offset_world_pilot/`).** It reuses the `confirmatory_v3` world, sensing,
friction and fault seeds of replicate 0 on purpose (paired replication of the
control cells); that overlap is deliberate and is why it cannot be pooled.

## Question

The planner predicts station visibility from a prior map that equals the
simulated world, and picks transition sites a median 5.2 cm (screen-shift
terms) from the edge of the elevated screen's shadow
(`MODULAR_NAV_AUDIT_2026-11.md` sections 4 and 6; `robust_site_wave2.md`). Under a
fixed screen offset of a few centimetres, is the `compact_to_narrow` transition
refused at docking and the mission lost, and does a planner prior-map margin of
4 cm remove that? No such failure has ever been executed in the simulator.

## Manipulations

- **World offset** (per trial, `world_offset` in the trial manifest parameters):
  the elevated sight screen is displaced in the simulator's SDF and in the
  manifest the fiducial station node loads. The planner reads the unperturbed
  prior (`world.prior.manifest.json`). The station is not displaced in this
  pilot (the field exists and is tested).
- **Direction rule (fixed in advance, `offset_world_pilot.py`).** The shadow
  of a screen at `SCREEN_FRACTION` 0.5 translates by 2t in the direction the
  screen translates by t, so the shadow moves toward the site exactly when the
  screen moves in the direction in which the site's pods first become hidden.
  For each layout, over the four axis directions, find the smallest screen
  translation (1 mm steps to 150 mm) that hides at least one moved pod at the
  shipped (margin 0) site with the core exactly on the site. "Toward" is the
  axis with the smallest such distance; "away" is its negation. Both margins of
  a layout share the direction, so they face the same world.
- **Conditions:** `ctl` (no offset), `t02` (2 cm toward), `t04` (4 cm toward),
  `a02` (2 cm away).
- **Planner margin m** in {0, 0.04} m: screen inflated by m on every side, and
  visibility also required from station poses offset by +-m per axis, on top of
  the shipped +-8 cm placement set. m = 0 is the shipped predictor.
- **Layouts (26):** the non-neutral `docking_observability` (14) and
  `combined_constraints` (12) layouts of `confirmatory_v3` on which the shipped
  treatment plans, read from the committed `manipulation_check.json`. Method
  `sensing_feasibility_coupled` only; one replicate; the 8 cells of a layout
  (2 margins x 4 conditions) share all four seeds (paired).

## Predictions (stated before any trial)

From `predictions.json`; verdict per cell = does the world's station see every
moved pod at the docked pose at the planned site (core exactly on the site),
plus a probability under the wave-2 placement model (1.1 cm per axis, 1.2 cm
bias in a random direction, 2 deg yaw; 400 draws per layout).

| cell type | predicted | expected refusals (placement model) |
|---|---|---|
| m=0, ctl / a02 | accepted, all 26 | 0 |
| m=0, t02 | accepted, all 26 (smallest escape distance 3.3 cm) | 0.2 |
| m=0, t04 | refused 4 (comb-03, comb-17, dock-02, dock-05), accepted 22; 16 cells marginal | 5.8; 90% interval 3 to 9; P(0 refusals) = 1.5e-4 |
| m=0.04, all conditions | accepted in 25; no plan on `docking_observability-17` (4 cells) | 0 of 25 |

Note the analysis predicts essentially no loss at 2 cm toward: sites keep at
least 3.3 cm of screen shift to the shadow edge, so loss starts between 3 and
5 cm of offset. The 2 cm cells are predicted-safe cells, not loss cells.

## Primary outcome

Per cell: **transition refused at docking** = the mission ended with
`terminal_status` in {`docking_failure`, `unsafe_topology`} or recorded
`reconfiguration_failures` >= 1. Secondary: mission completion
(`completed`); the planned site against `predictions.json`; `planning_failure`
(expected only for the 4 `docking_observability-17` margin cells); simulated
duration. Other failures (`controller_failure`, `timeout`, ...) are reported as
unrelated and are not counted as refusals. `score` (below) computes all of it.

## What would falsify the analysis (decision rules, fixed now)

1. **Predicted-safe cells fail.** 178 cells are predicted accepted outside
   m=0/t04. The placement model expects 0.2 refusals (95th percentile 1, 99th
   percentile 2). More than 1 refusal among them falsifies the geometric
   prediction. Any non-completion of the 26 `m=0, ctl` cells, which replicate
   the `confirmatory_v3` replicate-0 treatment trials (all completed there),
   halts the pilot as a platform regression, not a finding.
2. **Predicted-loss cells do not fail.** Among the 26 `m=0, t04` cells, fewer
   than 3 refusals (below the model's 5th percentile; zero has probability
   1.5e-4) falsifies the claim that the planner sits on the shadow edge at the
   stated margin. 3 to 9 refusals is consistent with the prediction; more than 9
   means the model understates loss and is also reported as a miss.
3. **Margin does not help.** More than 0 refusals among the 25 `m=0.04, t04`
   cells falsifies "a 4 cm prior-map margin removes the loss at a 4 cm offset".
   The paired contrast (per layout, m=0 vs m=0.04 at t04) is reported as counts
   of discordant pairs; no test is run at this size.
4. **Wrong cells.** Of the 4 nominally refused cells, completions are
   unremarkable (their probability of acceptance is 0.09 to 0.47) and are listed,
   not used to decide.

Verdicts are produced by `python3 -m modular_robot_benchmarks.offset_world_pilot
--score <raw dir>` and are reported as predicted-versus-observed tables with the
exact counts; a rule that cannot be decided (cells missing) is reported
undecided.

## Stop rules

- After the smoke subset, stop and investigate (do not start the full pilot) if
  any smoke `m=0, ctl` trial does not complete or plans a different site than
  `predictions.json`; if a trial's artifacts lack the declared `world_offset` or
  its `world.prior.manifest.json` differs from the zero-offset manifest; if
  `score` reports `site_mismatch`; or if more than one smoke trial ends in an
  infrastructure failure.
- Stop the full pilot if more than 3 of the first 40 trials fail for
  infrastructure reasons, or the clone moves off the pilot commit. Never edit
  the design, the predictions or the rules above after a record exists; a design
  change is a new design ID.
- No early stopping on outcome and no added trials: the pilot is the 208 trials.

## Cost (from `confirmatory_v3` records)

Measured on the 26 completed replicate-0 treatment trials of these same layouts
in `confirmatory_v3`: mean 259 s wall per trial (median 236 s, max 479 s),
setup through teardown. A refused transition ends the mission earlier (about
74 s mean in `confirmatory_v3` for docking refusals), planning failures about
12 s. The campaign overall ran at 30 to 35 trials per hour (all methods).
Estimate, one container, serial:

- **Smoke (12 trials):** about 0.9 h (11 completions at 259 s plus one early
  refusal), 1.1 h with a single infrastructure retry.
- **Full (208 trials; the smoke 12 are inside it):** about 14.5 h expected
  (about 198 completions, about 6 refusals, 4 planning failures), 15 h upper
  bound at 259 s for every trial, about 16 h with the usual 5 to 10% retry
  overhead. The remaining 196 trials after smoke are about 13.7 h.

## Commands

The earlier campaign scripts (`start_campaign`, `resume_campaign.sh`,
`results/run_<study>.sh`) live in the WSL clone's unversioned `results/`
directory, not in this repository. `tools/run_offset_world_pilot.sh` is the
closest equivalent, written from `AGENTS.md` and `tools/finish_confirmatory_v3.sh`;
its assumptions (container `morphology_navigation_dev`, clone
`/home/checker/morphology_ws` = `/home/roboboat/morphology_ws`, `colcon build`
of the four touched packages, container image passed as
`MORPHOLOGY_CONTAINER_IMAGE`, `GZ_IP=127.0.0.1`) are stated in its header. In
WSL, with the clone checked out clean at the proposal branch tip `<commit>`:

```
bash tools/run_offset_world_pilot.sh smoke <commit>   # 12 trials, ~1 h
tail -f /home/checker/morphology_ws/results/run_offset_world_pilot_smoke.launch.log
# ends with: offset_world_pilot smoke done
python3 -m modular_robot_benchmarks.offset_world_pilot \
    --out studies/offset_world_pilot_v1 --score results/offset_world_pilot/raw
# apply the smoke stop rules; only then:
bash tools/run_offset_world_pilot.sh full <commit>    # resumes; skips the 12 done
# ends with: offset_world_pilot full done
python3 -m modular_robot_benchmarks.offset_world_pilot \
    --out studies/offset_world_pilot_v1 --score results/offset_world_pilot/raw
```

Resume after an interruption by re-running the same command (finished records
are kept). Spot-check one offset trial's artifacts before trusting the batch:

```
A=results/offset_world_pilot/artifacts/<trial id>
grep -A5 world_offset $A/world.manifest.json
diff <(jq 'del(.role,.physical_disturbance,.friction_seed)' $A/world.prior.manifest.json) \
     <(jq 'del(.world_offset,.physical_disturbance,.friction_seed)' $A/world.manifest.json)   # screen centre only
```

`study status` / `analyze` do not apply to this design (it is not a
four-method study design) and must not be run on it.
