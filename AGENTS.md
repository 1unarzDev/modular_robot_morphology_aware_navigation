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

Gate 0 mechanics pass on the engineering layout: round trips complete with
signed-motion and all-pod rigidity gates, and the seven-case fault matrix
passes once. Transition feasibility now also sweeps the post-transition
verification maneuver (ADR 0003), which closed the failure the workshop
diagnostic exposed.

That change invalidated the `combined_constraints` Gate 2 golden layout, which
was re-selected to world seed 1 against a recorded seed sweep in the roadmap.
That family separates on only a third of sampled seeds and needs confirming
across the frozen design's layouts. See `docs/current_status.md` "Resume here"
for the three open items.
