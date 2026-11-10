# predetach_check_v1: runbook (job queue)

Read `declaration.md` first (predictions, interim look, stop rules). This file
says how to run it. Design hash and trial counts are in `predictions.json`
(`design_hash`, `tiers`, `estimated_hours`). Nothing here pushes anywhere.

## 1. Job shape (backend `mrmn_trial`)

The backend runs one trial per seed, where **seed = `run_order`**, from a
clean clone at a given commit. It builds the **whole** workspace with colcon
(this branch changes `modular_robot_msgs`: `ComputeHybridPlan.action` gained
`site_exclusions_json`, so a partial `--packages-select` build without the
message package will fail at the first planner call), then invokes the batch
module directly; `run_morphology_missions` is not on PATH in a fresh install.

```json
{"schema": "rc-job/1", "job_id": "<id>", "kind": "mrmn_trial",
 "seeds": [0, 1, 2, ...],
 "repos": [{"name": "mrmn", "url": "/home/lunarz/modular_robot_morphology_aware_navigation",
            "ref": "<full commit of proposal/predetach-check>"}],
 "spec": {"design": "studies/predetach_check_v1/design.json",
          "campaign": "predetach_check_v1"}}
```

The branch is local: the `url` repository must contain the commit (fetch the
branch into it, or bundle it as the offset pilot's runbook does; never push).

Per seed `k`, the trial is the unique spec with `run_order == k`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build                                  # whole workspace
source install/setup.bash
TID=$(python3 -c "import json,sys; d=json.load(open('studies/predetach_check_v1/design.json'))['design'];
print(next(t['trial_id'] for t in d['trials'] if t['run_order']==$k))")
echo "[\"$TID\"]" > only.json
GZ_IP=127.0.0.1 python3 -m modular_robot_benchmarks.mission_batch \
  --design studies/predetach_check_v1/design.json \
  --raw results/predetach_check_v1/raw --artifacts results/predetach_check_v1/artifacts \
  --only only.json
```

`--only` and the `run_order` sort behave exactly as on the offset pilot
(`mission_batch.pending_trials`): unknown ids are an error, finished records
are skipped, so re-running a seed is idempotent. The design's trials are
stored in `run_order` order, so `trials[k]` is also seed `k`.

Smoke: `studies/predetach_check_v1/smoke_trial_ids.json` = seeds `0..9`.
Tier boundaries are in `predictions.json["tiers"]`; the interim look is after
the last seed of tier 1.

## 2. Verify the check executed (orchestrator, per record)

Record `results/predetach_check_v1/raw/<trial_id>.json`:

- `.spec.method`, `.spec.exclusion_rule`, `.spec.run_order`, `.spec.arm`.
- `.manifest.parameters.predetach` = `{exclusion_rule, arm, source_trial_id,
  run_order}` (present only for this design).
- `.controller_diagnostics.predetach_checks`: the navigator's ledger, a list of
  events. For `feasibility_checked`/`feasibility_probed` it **must** start with
  `{"event": "start", "parameters": {"rule": <spec.exclusion_rule>, "dwell_s":
  1.0, ...}}`. Each `{"event": "check"}` carries `site`, `actual_pose`,
  per-pod `pods.<pod>.{seen, fresh_samples, visible_samples}`,
  `unseen_pods`, `probes` (probed variant), `passed`. Each failure is followed
  by `{"event": "rejection"}` and either `{"event": "replan"}` or a `final`
  with `outcome: attempt_cap`. `plan` events list the sites each replan chose
  (`rejections_applied`). The last event is `final` with `outcome` in
  {completed, no_plan, attempt_cap, docking_refused}, unless the mission hit
  the 300 s deadline (then there is no `final`; `terminal_status` says
  timeout).
- Missing `predetach_checks` on a checked trial, or a `start` whose rule
  differs from the spec, means the check did not run: stop and diagnose.
- `.transition_edge_decisions[].reasons` contains `predetach_excluded:<rule>`
  after the first rejection (the planner applied it).
- Outcome: `.terminal_status`, `.completed`, `.simulated_duration_s`,
  `.reconfiguration_failures`, `.planned_transition_sites` (the final plan's
  sites).
- Reference methods (`feasibility_coupled`, `sensing_feasibility_coupled`)
  must have **no** `predetach_checks` key.

Scoring against the predictions:
`python3 -m modular_robot_benchmarks.predetach_check --score results/predetach_check_v1/raw`.

## 3. First things to read in the smoke logs

The ROS side of this branch could not be run where it was written. In
`results/predetach_check_v1/artifacts/<trial>/launch.log`:

1. `predetach start:` from `hybrid_navigator` with the right rule (the launch
   argument `predetach_exclusion_rule:=` is in the command line of every
   checked trial).
2. `predetach check:` lines: are `fresh_samples` > 0 for every moved pod? Zero
   fresh samples for all pods means the estimate stamps and the navigator's
   clock disagree (both should be sim time) — every check would then fail
   closed and the robot would reject every site.
3. The planner accepted `site_exclusions_json` (no "unknown field"/ValueError
   message in the plan result; a "no collision-free hybrid plan" after
   rejections is a legitimate outcome).
4. Probed variant: the site alignment to each probe pose (`could not align`
   errors), and that the robot returned to the site before transforming.
5. `fiducial_station` still publishing while the robot dwells (the check
   relies on its `/fiducials/<pod>/pose` stream).
