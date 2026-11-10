# expchoice_doorway_v0

**Exploratory.** A separate research track (ExperimentChoice, in the CRANE
repository) borrowing this repository's simulator and trial runner. Nothing
here is evidence for the morphology study; its designs, records and hashes are
untouched. No trial of this design has been run.

Question: for the compact differential robot at an offset doorway with an
approach bay of depth d, do two frozen Nav2 configurations swap rank as d,
the lateral offset e and the door width w vary?

| File | What |
|---|---|
| `courses/course_<id>.json` | ExperimentChoice truth manifests (`expchoice.course/v0`), copied byte for byte from `Tools/ExperimentChoice/nav2/courses_v0` |
| `design.json` | Frozen design: 10 courses x arms A, B x 2 replicates = 40 trials, `run_order` 0-39. Hash `567e4bd026a1ef155fc57b976f48fc09042c855599bdc86d9edf5b912c492bb8` |
| `canary_trial_ids.json` | `run_order` 0 and 1: both arms on the roomiest course (d = 1.20 m) |
| `NAV2_CONFIGS.md` | The frozen parameter files, every difference between arms, confounds, unverified points |

Rebuild (must reproduce `design.json` exactly, or it refuses):

```
python3 -m modular_robot_benchmarks.doorway_design \
    --manifests studies/expchoice_doorway_v0/courses/*.json \
    --canary-course 8d876a63964f9337 --replicates 2 \
    --out studies/expchoice_doorway_v0/design.json
```

Run one trial (inside the image, built workspace; the job queue's
`mrmn_trial` adapter does exactly this, one `run_order` per seed):

```
python3 -m modular_robot_benchmarks.mission_batch \
    --design studies/expchoice_doorway_v0/design.json \
    --raw <raw dir> --artifacts <artifact dir> --only <json list of trial ids>
```

## How a doorway trial differs from every other trial here

| | Other families | This family |
|---|---|---|
| Scenario | `make_confirmatory_scenario`: walls are cells of a 0.1 m grid drawn from a seed | `doorway_scenarios.DoorwayCourse`: the manifest's integer centimetres, carried in the trial spec and covered by the design hash |
| World | one box per occupied cell | one box per wall, exact |
| Map | grid cells, 4x upsampled (0.025 m) | exact walls rasterised at 0.01 m |
| Keepout mask | always served | none |
| Driver | `navigate_hybrid` (morphology planner + `FollowPath` by controller id) | stock Nav2 `navigate_to_pose` (planner server, controller server, default tree) |
| Stack file | `config/nav2.yaml` | a frozen file per trial (`nav2_params` launch argument, default unchanged) |
| Deadline | 300 s simulated | 120 s simulated |
| Outcome | status from the navigator's message | status from the action's goal status and error code; evaluator contact and localization rules unchanged; extra block `controller_diagnostics.expchoice` |

## Geometry and cell size

The manifest, the world and the evaluator agree to the millimetre for any
course. The map agrees to the centimetre when `w_cm` and `B_cm` are even
(every edge is then a whole centimetre); an odd value puts the jambs on the
half centimetre and the map's opening is 1 cm narrower than the world's. The
design builder refuses such a course unless told otherwise. Tests read door
width, offset, bay depth and wall thickness back off the written SDF and PGM.

What else depends on a cell size:

- **Global costmap**: its static layer adopts the map's resolution, so it runs
  at 0.01 m here (0.025 m in the other families), about 0.3 million cells. Cost to
  be checked in the canary.
- **Arm C's planner** downsamples that 5x to 0.05 m. Exact when every edge is
  on the 5 cm lattice (true for all ten courses of this design; the map origin
  and size are aligned for it), otherwise conservative by under 5 cm a face.
- **Local costmap** 0.025 m, rolling in the odometry frame: unchanged, and not
  aligned to walls in any family.
- **AMCL**'s likelihood field is built from the map's cells; `sigma_hit` 0.05
  was tuned on 0.025 m cells.
- **Evaluator contact**: `clearance_metrics` ignores obstacles whose centre is
  more than 1 m from a module, sized for per-cell boxes. The walls are passed
  to it cut into tiles of at most 0.2 m (their union is exactly the wall).
- **Morphology planner** (0.1 m planning grid): still launched, receives the
  map and coarsens it, and is never asked to plan.
- **Start and goal** are the manifest's centimetres, not cell centres.
