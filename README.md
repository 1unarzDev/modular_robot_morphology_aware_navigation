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
to its catalog pose using world-pose feedback, verifies the detachable joint
after relatching, and commits the new morphology atomically. A failed or
cancelled transition leaves the manager's morphology unchanged.

The hybrid planner searches `(x, y, heading, morphology)` and accounts for
traversal time, energy, failure probability, unknown-space exposure, oriented
footprint collision, locomotion constraints, and reconfiguration swept-space
clearance. The ROS adapter conservatively downsamples the SLAM map to 0.1 m for
responsive global search while Nav2 retains its 0.05 m execution costmaps.

## Verified behavior

- Host planner and control tests: `python3 -m pytest -q` (16 passing).
- Container build: all nine ROS packages build with `colcon build`.
- Container package smoke tests: all nine packages pass `colcon test`.
- Gazebo exposes lidar, RGB-D, IMU, odometry, TF, six pod command topics, joint
  acknowledgements, and module world-pose feedback.
- A positive Nav2 command moves the complete latched assembly through physical
  wheel forces from its pods.
- `compact_to_ackermann` physically moved and relatched four pods, then changed
  the authoritative morphology after all acknowledgements. The observed run
  took 13.9 s and final pod positions were within about 2.4 cm of targets.
- A live SLAM-map plan through the 0.42 m doorway contained compact traversal,
  `compact_to_narrow`, and narrow traversal. It expanded 51,886 hybrid states.
- A top-level `NavigateHybrid` traversal completed through Nav2 in 41.6 s.

## Research limitations

The detachable joints and pod locomotion use Gazebo physics, but the magnetic
connector geometry, contact-guided final insertion, battery dynamics, and spine
actuation remain idealized. Ackermann and articulated behavior are expressed by
planner constraints and morphology-specific MPPI models; the pods themselves
are fixed-angle differential units after docking. The sample cost-observation
CSV is synthetic scaffolding and must be replaced with logged randomized trials
before reporting learned-model results. Full doorway-crossing success under
online SLAM corrections remains an evaluation target rather than a verified
result.
