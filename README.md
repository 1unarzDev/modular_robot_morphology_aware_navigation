# Modular Robot Morphology-Aware Navigation

Research platform for navigation that plans jointly over robot pose and a finite
catalog of physically validated modular morphologies. The reference robot uses a
fixed inventory of six self-mobile steer-drive pods that detach, relocate, and
redock around an articulated sensor core.

The repository is a ROS 2 Jazzy workspace, but the planner core and its tests can
also run with ordinary Python 3.

## Packages

- `modular_robot_msgs`: ROS messages, services, and actions for topology and
  hybrid plans.
- `morphology_planner`: morphology-augmented lattice planner and ROS action
  adapter.
- `morphology_manager`: authoritative topology state and atomic morphology-mode
  changes.
- `reconfiguration_executor`: staged undock, relocate, align, latch, and verify
  executor.
- `modular_robot_description`: canonical module/morphology/transition catalog and
  Xacro robot description.
- `modular_robot_sim`: Gazebo worlds and simulation launch files.
- `modular_robot_bringup`: integrated SLAM/Nav2 launch and configuration.
- `modular_robot_benchmarks`: deterministic scenario generation and experiment
  runner.

## Quick checks without ROS

```bash
python3 -m pytest -q
PYTHONPATH=src/morphology_planner python3 -m morphology_planner.demo
```

## ROS 2 development environment

The devcontainer reuses the published `astro_dock` ROS 2 Jazzy images. On this
host:

```bash
.devcontainer/prebuild.sh
devcontainer up --workspace-folder .
devcontainer exec --workspace-folder . bash -lc \
  'rosdep install --from-paths src --ignore-src -y && colcon build --symlink-install'
```

Launch the deterministic demonstration after building:

```bash
source install/setup.bash
ros2 launch modular_robot_bringup demo.launch.py
```

The scientific scope, prior art, and evaluation protocol are recorded under
[`docs/research`](docs/research/literature_review.md).

## Implemented end-to-end path

`NavigateHybrid` obtains the current `map -> core/base_link` pose, requests a
weighted hybrid-A* plan, sends traversal segments to the matching Nav2 MPPI
controller, and sends topology edges to the reconfiguration action. Nav2's
collision-checked body twist is projected through the active morphology's pod
geometry and applied by the six simulated differential-drive pods.

During reconfiguration, assembled motion is inhibited and pods move one at a
time. The executor waits for Gazebo to confirm detachment, drives the free pod
from topology-anchored wheel odometry and visibility-limited connector-camera
estimates, verifies the detachable joint after relatching, and commits the new
morphology only when observed topology matches the target. A failed or
cancelled transition enters `RECOVERY_REQUIRED`; assembled drive remains
inhibited until the observed topology is reconciled.

The hybrid planner searches `(x, y, heading, morphology)` and accounts for
traversal time, energy, failure probability, unknown-space exposure, oriented
footprint collision, locomotion constraints, and reconfiguration swept-space
clearance. Four explicit study methods separate route-first adaptation,
geometry-only coupling, full transition feasibility, and sensing-aware
feasibility. Plans carry map, topology, and sensing-signature revisions. The ROS
adapter conservatively downsamples the SLAM map to 0.1 m for
responsive global search while Nav2 retains its 0.05 m execution costmaps.

## Verified behavior

- Host planner, safety, sensing, validation, and study tests:
  `python3 -m pytest -q` (67 passing in 70.00 s on 2026-09-12).
- Container build: all nine ROS packages build with `colcon build`.
- Container package smoke tests: all nine packages pass `colcon test`.
- Gazebo exposes lidar, RGB-D, IMU, odometry, TF, six pod command topics, joint
  acknowledgements, and module world-pose feedback.
- A positive Nav2 command moves the complete latched assembly through physical
  wheel forces from its pods.
- An earlier `compact_to_ackermann` engineering run physically moved and
  relatched four pods. This morphology is now excluded from confirmatory study
  claims because the assembled fixed-angle pod mechanics do not implement true
  Ackermann steering.
- A live SLAM-map plan through the 0.42 m doorway contained compact traversal,
  `compact_to_narrow`, and narrow traversal. It expanded 51,886 hybrid states.
- A top-level `NavigateHybrid` traversal completed through Nav2 in 41.6 s.

## Research limitations

The detachable joints and pod locomotion use Gazebo physics, but connector
cameras, magnetic capture, contact-guided final insertion, battery dynamics,
and spine actuation remain idealized. Ackermann, omni, crawler, articulated,
and stacking entries are future concepts rather than study conditions. The
sample cost-observation CSV is synthetic scaffolding and is excluded from the
confirmatory pipeline. A sensor-driven `compact_to_narrow` transition now
completes, while the reverse qualification still fails intermittently during a
pod relocation, so no confirmatory mission results are claimed. Full
doorway-crossing success under sensor-based localization and
online SLAM corrections remains an evaluation gate.

## Study commands

The frozen confirmatory schedule contains 432 paired terminal trials. The
analysis refuses incomplete or mixed-manifest data and writes CSV, Markdown,
JSON, and dependency-free SVG figures.

```bash
morphology_study status \
  --design studies/confirmatory/design.json \
  --raw results/confirmatory/raw

morphology_study analyze \
  --design studies/confirmatory/design.json \
  --raw results/confirmatory/raw \
  --output results/confirmatory/derived
```

See [`docs/research/conference_roadmap.md`](docs/research/conference_roadmap.md)
for the evidence gates and [`docs/research/statistical_analysis_plan.md`](docs/research/statistical_analysis_plan.md)
for the estimands and inference procedure.
