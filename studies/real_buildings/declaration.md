# Exploratory real-buildings study: how often does an omitted transition condition bind in scanned buildings?

Declared under the author's standing delegation. **Exploratory**: it is not
part of the confirmatory study, receives no hypothesis test, and changes
nothing in `confirmatory_v3`'s design, analysis or interpretation rules.

## Why

The generated-room prevalence study (`studies/prevalence/`, 300 layouts) put
the rate at which a control's transition site fails the full test at 5.4%
[3.3%, 8.5%], every failure an unobserved station, but "for this generator,
not for real buildings". This study asks the same planner-level question of
scanned real buildings: in missions drawn from a building's own free space
that the robot can only complete by changing shape, how often is the site a
control chooses one the full transition test rejects?

## Population

HM3D v0.2 (Habitat-Matterport 3D Research Dataset; Ramakrishnan et al., 2021),
the `train` and `val` splits, GLB format: 900 scanned real buildings (homes,
offices, shops). Used under Matterport's academic, non-commercial terms; only
derived planner records are kept, never meshes. The 10 `minival` buildings
were used to develop the pipeline (below) and are **not** in the sample.

The sample is every building in the two splits, in the order of
`studies/real_buildings/sample.txt` (sorted by HM3D id), committed with this
file. No building is excluded or resampled; a building whose processing
raises is recorded as an error, and reported.

## From scan to world model (`modular_robot_benchmarks.building_mesh`)

Geometry only; HM3D GLBs carry no object labels besides ceilings, which are
dropped.

- Surface sampled at 2500 points/m^2 (seed 0) plus every vertex.
- Floor levels: 5 cm bands holding >= 8 m^2 of upward-facing area
  (normal z > 0.85), >= 1.8 m apart.
- Grid, 0.10 m cells, per level: a cell is free only if observed floor
  (upward samples within 0.05 m of the level) lies in it and no surface lies
  0.05--0.14 m above the floor. Unobserved floor is occupied. The 0.12 m lidar
  plane lies in that band; the band also maps low geometry, which the
  generated worlds never had (see the module docstring).
- 3D boxes: surface 0.05--2.0 m above the floor, voxelized at 0.10 x 0.10 x
  0.05 m and greedily merged; voxels over grid-occupied and over free cells
  are merged separately. Every box blocks a station sight line. Only boxes
  over free cells -- what the scan plane misses, i.e. raised furniture --
  enter module sweeps, the verification sweep and driving: in every generated
  layout walls were grid-only, and the 1.0 m planar clearance disk is what
  keeps a transition off grid-mapped geometry (catalog,
  `post_transition_verification`).

## Missions (`modular_robot_benchmarks.real_buildings`)

Per building, a `random.Random` seeded from SHA-256 of
`"<master seed>:<building id>"`, master seed **20261115**. On the planner's own
lattice (8 headings, forward-only moves and in-place turns, as
`lattice_reach` reproduces from `MorphologyAStar`):

- **start**: a uniformly drawn (cell, heading), over all levels, where the
  compact body is free;
- **goal**: a cell drawn uniformly among those the narrow body reaches from the
  start through a geometry-feasible `compact_to_narrow` transition, in 20--250
  lattice steps, that the compact body reaches at no heading;
- **passage**: the first cell on a shortest lattice route that the compact
  body cannot reach;
- **station**: a fiducial station at 1.80 m above the floor, on a uniformly
  drawn free cell that is 4-adjacent to an occupied cell, within 0.5 m
  (5 cells, through free cells only) of the compact-reachable cells and
  3.0 m of the passage, with no scanned surface in its cell at 1.75--1.90 m;
- **world**: the grid and boxes of a window around the route and station,
  with a 3.0 m margin.

A drawn mission enters the sample only if `geometry_coupled` plans it.
`geometry_coupled` checks least, so where it cannot plan no coupled method
can, and the draw is no mission. Up to 3 missions per building, from up to
400 draws; a building may contribute none (no mission needs a shape change,
or none was found), which is counted.

## Procedure

As in the generated-room study: the planner alone (heading bins 8, epsilon
2.5; no simulation and so no robot outcome) plans `geometry_coupled`,
`feasibility_coupled` and `sensing_feasibility_coupled` with the mission's
station predictor. Every transition a control plans is re-tested under the
full transition test (planar, module trajectories, verification maneuver,
predicted observability), and every failing stage is recorded.

## Reported, and nothing else

For each control: among missions where it plans a transition, the fraction in
which the full test rejects a site it plans,

1. over each building's **first** mission, with a 95% Wilson interval
   (primary: one mission per building, independent);
2. over **all** missions, with a 95% building-cluster bootstrap interval
   (2000 resamples, seed 0);

the rejections by failing stage; how many rejected missions the treatment
still plans; and the missions each method cannot plan. Also, descriptively:
the buildings with at least one mission, the draw counts (no goal, no
station, geometry unplanned), and processing errors. No building or mission
is excluded after results, the sample size is not changed, and no constant
above is changed after the sample is evaluated.

## Disclosed before the sample

Development on the 10 `minival` buildings only (master seed 1):
- The first sampler approximated reachability with "the footprint fits at
  some heading", which drew starts the forward-only planner cannot leave
  (every mission `NoPathError`). It was replaced by the lattice reproduction,
  checked against `_traversal_is_free` (8 of 987 random moves differ, all
  floating-point ties where the mask is the stricter) and against the
  transition disk (ties included, so the mask is again the stricter).
- Low geometry (0.03--0.10 m) was first left unmapped; it filled free floor
  with drive obstacles, and was moved into the grid band.
- Grid-mapped boxes (walls) first entered module and verification sweeps.
  Every verification rejection that produced was a wall-base box 0.95 m from
  the site, just outside the 1.0 m planar disk the platform relies on for
  walls; that is a different test from the one the study runs, so mapped
  boxes now only block sight lines.
- The station's 0.5 m reach was first a plain dilation, which can cross a
  0.1--0.3 m interior wall; it now grows through free cells only.
- The planner's sight-line and swept-volume scans, and its drive-obstacle
  test, were sped up by exact pre-filters (`spatial_index`, tests in
  `tests/test_spatial_index.py`); no planner decision changes.
- **Development outcomes seen** (minival, master seed 1, after every change
  above; 9 of 10 buildings had a mission, 27 missions): the full test
  rejected a site of `geometry_coupled` in 7 of 27 missions and of
  `feasibility_coupled` in 7 of 27, every one at observability, in 3
  buildings; the treatment could not plan 3. Earlier development runs, with
  the defects above, were also seen. None of this is sample evidence. The
  sample size, the two estimands and every other constant were fixed before
  these outcomes and not changed after them; the two corrections above were
  each made on a traced artifact, not on a rate.

## Run

    .venv-host/Scripts/python tools/run_real_buildings_host.py \
        --buildings studies/real_buildings/sample.txt \
        --data-root C:/Users/risha/datasets/hm3d --master-seed 20261115 \
        --workers 7 --output studies/real_buildings/real_buildings_20261115.json

Resumable: a per-building checkpoint
(`real_buildings_20261115.json.partial.jsonl`) is kept beside the output and
is not committed.


## What it can claim

A planner-level rate on scanned real buildings: how often a control's site,
in missions that need a shape change, is one the full test says will collide
in a module or verification sweep or be refused docking. It inherits the
scans' limits: floor never scanned (under beds and sofas) is occupied, so
raised furniture there is mapped rather than raised, and the
raised-obstacle stage is a lower bound; stations are placed by the rule
above, not found in the buildings; the robot is the study's two-morphology
platform. It is not a rate of robot failures.
