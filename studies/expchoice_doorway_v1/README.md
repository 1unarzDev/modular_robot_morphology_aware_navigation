# expchoice_doorway_v1

**Exploratory; no trial has been run.** Replaces `expchoice_doorway_v0`, which
was never run. Same runner, same frozen Nav2 files (arms A and B), same design
kind (`expchoice_doorway_v0` names the scenario family and runner, not this
batch), same door width (0.90 m), wall thickness, deadline and outcome rules.
Read `../expchoice_doorway_v0/README.md` and `NAV2_CONFIGS.md` for those.

| File | What |
|---|---|
| `courses/course_<id>.json` | ten ExperimentChoice manifests: v0 courses 2, 5, 8, 9, 10 and (d, e) = (1.20, 0.45), (1.30, 0.65), (1.45, 0.85), (1.30, 0.45), (1.60, 0.20) m |
| `design.json` | 10 courses x arms A, B x 2 replicates = 40 trials, `run_order` 0-39, hash `216803d711c2ac34a930798b51a4651a152a1b2274932e984cea5730e21d6f0e` |
| `canary_trial_ids.json` | `run_order` 0 (arm B) and 1 (arm A) on the roomiest course (d = 1.60 m, e = 0.20 m) |

All ten courses are inside the frozen admissible box and have every wall edge
on the 5 cm lattice. Trials of the five shared courses carry the same ids and
plant seeds as in v0; the design hash and the campaign differ.

```
python3 -m modular_robot_benchmarks.doorway_design \
    --manifests studies/expchoice_doorway_v1/courses/*.json \
    --canary-course a96f269f80ad17d1 --replicates 2 \
    --out studies/expchoice_doorway_v1/design.json
```
