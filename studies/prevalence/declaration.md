# Exploratory prevalence study: how often does an omitted transition condition bind?

Declared 2026-11-04, before any layout of the declared sample was generated,
under the author's standing delegation. **Exploratory**: it is not part of the
confirmatory study, receives no hypothesis test, and changes nothing in
`confirmatory_v3`'s design, analysis or interpretation rules.

## Why

The confirmatory families are constructed so that each omitted condition can
matter. They measure what the information is worth *when* a raised obstacle, a
verification sweep or an occluded station constrains the staging area. They
cannot say how often such a constraint arises in rooms not built to produce it.
This study estimates that, on a population that targets no method.

## Population

`modular_robot_benchmarks.prevalence.make_prevalence_layout`, at the commit that
adds this file:

- the generator-3 room (`_wide_layout`, ADR 0013): 7.1--8.0 x 3.2--4.0 m arena,
  one interior wall with a 0.70 m doorway, start and goal off the door line;
- furniture in the start room, each piece backed onto one of its four walls at
  a uniform position along it, never within 0.5 m of the spawn or the doorway:
  - 0, 1 or 2 tall floor-standing pieces (uniform), 0.3--0.9 m long, 0.2--0.5 m
    deep, 1.0--2.0 m high, entered in the 2D map and the 3D environment;
  - 0--3 raised pieces (uniform), 0.3--1.0 m long, 0.1--0.4 m deep, underside
    0.13--0.26 m (above the 0.12 m lidar plane, below the 0.30 m tallest body),
    0.03--0.10 m thick, in the 3D environment only;
- one fiducial station at 1.8 m on the south, north or west wall (uniform), at
  a uniform position along it.

**Disclosed before the sample:** a smoke test on master seed 1 (not the sample)
placed furniture uniformly free-standing; in a 3.9 x 3.3 m start room that left
no 1 m transition disk for any method in either of 2 layouts, so placement was
changed to wall-backed. Six further master-seed-1 layouts then planned for every
method. Only whether each method planned was looked at, never a site re-test.

## Sample and procedure

300 layouts, world seeds drawn from master seed **20261104**. For each layout the
planner alone (heading bins 8, epsilon 2.5, as in missions; no simulation and so
no robot outcome) plans `geometry_coupled`, `feasibility_coupled` and
`sensing_feasibility_coupled` with the mission's station predictor. Each
control's chosen `compact_to_narrow` site is then re-tested under the full
transition test (planar, module trajectories, verification maneuver, predicted
observability), and every failing stage is recorded.

## Reported, and nothing else

For each control, among layouts where it plans a transition: the fraction whose
site the full test rejects, with a 95% Wilson interval; the count by failing
stage; and how many of those layouts the treatment still plans. Also the number
of layouts each method cannot plan, and the geometry rejection count by
furniture count (descriptive). No layout is excluded or resampled, the sample
size is not changed after results, and the generator is not changed after the
sample is generated.

## What it can claim

A planner-level rate: how often the site a method chooses, on this population,
is one the full test says will collide in a module or verification sweep or be
refused docking. The confirmatory study supplies what such a site costs when
executed. It is a rate for this generator, not for real buildings.
