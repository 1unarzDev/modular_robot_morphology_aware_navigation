# Morphology-Aware Navigation for Modular Robots: Evidence and Study Gap

## Conclusion

The novelty claim cannot be that no planner searches over position and
morphology. hTetro work already selects discrete shapes during coverage and
jointly optimizes actions, paths, and configurations. Variable-footprint mobile
bases have also coupled navigation and reconfiguration with model-predictive
control. Autonomous modular-robot configuration and reconfiguration frameworks
also predate this study. Do not claim the first morphology-aware navigation
stack or the first ROS-based autonomous reconfiguration system.

The proposed contribution is **joint route and morphology selection conditioned
on environmental reconfiguration feasibility, sensing constraints, and
time/energy/risk costs**. ROS integration and typed module graphs support the
study but do not establish novelty by themselves. The evidence must show when
these additional constraints change decisions and improve mission outcomes
relative to sequential and geometry-only coupled planning. This is a bounded
research hypothesis, not a verified claim that no prior system does it.

## Closest prior art

| Area | Result | Remaining limitation for this study | Primary source |
|---|---|---|---|
| Joint path and morphology | Modified A* selects among hTetro's four morphologies to cover narrow regions. | Platform-specific planar coverage with a small discrete shape set. | Le et al., *Sensors* 2018, [doi:10.3390/s18082585](https://doi.org/10.3390/s18082585) |
| Joint path and morphology | A multi-objective genetic algorithm plans paths for a hinged-tetro reconfigurable tiling robot. | Coverage/tiling is a different objective from uncertain point-to-point navigation with physical docking; detailed capability comparisons require full-text screening. | Cheng et al., *IEEE Access* 2020, [doi:10.1109/ACCESS.2020.3006579](https://doi.org/10.1109/ACCESS.2020.3006579) |
| Coverage and reconfiguration | Graph search and dynamic programming combine coverage with hTetro morphology shifts. | Complete-coverage objective on one platform. | Le et al., *IEEE Access* 2019, [doi:10.1109/ACCESS.2019.2928467](https://doi.org/10.1109/ACCESS.2019.2928467) |
| Continuous footprint control | MPC supports simultaneous driving and reconfiguration of a steerable robotic base. | Continuously variable, fixed-topology base; a strong reference for feasible assembled motion. | Pankert et al., *Design and Motion Planning for a Reconfigurable Robotic Base*, *RA-L* 7(4):9012–9019, 2022, [doi:10.1109/LRA.2022.3189166](https://doi.org/10.1109/LRA.2022.3189166), [author code](https://github.com/leggedrobotics/swerve_steering/tree/release) |
| Integrated modular autonomy | Perception-driven autonomy selects configurations to match capabilities to encountered environments. | Closest architectural precedent; physical detach-drive-redock and autonomous configuration must not be presented as new. Full-text comparison of route costs and sensing constraints remains required. | Daudelin et al., *An integrated system for perception-driven autonomy with modular robots*, *Science Robotics*, 2018, [doi:10.1126/scirobotics.aat4983](https://doi.org/10.1126/scirobotics.aat4983) |
| Skeletal kinematics and onboard sensing | FreeSN uses skeletal kinematics, configuration matching/mapping, behavior libraries, and onboard sensing for autonomous 3D locomotion and reconfiguration with up to 18 modules and 48 joint motors. | The demonstrated configuration-library retrieval and control problem differs from jointly selecting environmental routes and transition locations under sensing uncertainty. Library-based transitions and sensor-driven topology estimation themselves are prior art. | Tu et al., *Locomotion and self-reconfiguration autonomy for spherical freeform modular robots*, online 2025, *IJRR* 45(3):477–500, print 2026, [doi:10.1177/02783649251360360](https://doi.org/10.1177/02783649251360360), [author manuscript](https://freeformrobotics.org/wp-content/uploads/2025/07/IJRR2025_MSRR_Autonomy.pdf) |
| Restricted-space platform | MIRRAX changes Mecanum-wheel geometry and demonstrates configuration-dependent controllability. | Platform/control contribution rather than route-dependent topology search. | West et al. 2022, [arXiv:2203.00337](https://arxiv.org/abs/2203.00337) |
| Mission-level selection | SMORES-EP maps task requirements to a library of configurations and behaviors; VSPARC transfers Unity designs to hardware. | Symbolic regions and stored behaviors rather than geometric/kinodynamic navigation. | Jing et al., RSS 2016, [doi:10.15607/RSS.2016.XII.025](https://doi.org/10.15607/RSS.2016.XII.025) |
| Morphology-general control | A graph-structured policy generalizes locomotion across simulated and physical module graphs. | Controls a supplied morphology; it does not decide where to reconfigure. | Whitman et al., *T-RO* 2023, [doi:10.1109/TRO.2023.3284362](https://doi.org/10.1109/TRO.2023.3284362) |
| Modular motion control | A QP framework adapts to changing kinematics and coupled goals on CKBot and SMORES-EP. | Local motion control without route-dependent topology selection. | Liu et al., IRC 2020, [doi:10.1109/IRC.2020.00008](https://doi.org/10.1109/IRC.2020.00008) |
| Reconfiguration planning | A two-layer planner generates module motions along a supplied trajectory. | Desired route/configuration is supplied rather than jointly selected. | Yoshida et al., *IJRR* 2002, [doi:10.1177/0278364902021010835](https://doi.org/10.1177/0278364902021010835) |
| Distributed reconfiguration | SMORES modules transform to a specified target configuration. | Target morphology is an input rather than a navigation decision. | Liu et al., *RA-L* 2019, [doi:10.1109/LRA.2019.2930432](https://doi.org/10.1109/LRA.2019.2930432) |
| Flexible connectors | FreeBOT modules move independently and attach magnetically at arbitrary spherical locations. | Connection hardware without an integrated morphology-aware navigation stack. | IROS 2020, [doi:10.1109/IROS45743.2020.9341129](https://doi.org/10.1109/IROS45743.2020.9341129) |

## IROS workshop evidence

Announced talks at the upcoming 2026 Modular Robot Workshop are discovery leads,
not completed experiments or published evidence. Cite an independently available
paper for a technical result, recording its actual publication status and date.

The official [IROS 2023 Tensegrity Robotics Workshop](https://www.eng.yale.edu/faboratory/tensegrityworkshop)
explicitly identified autonomous navigation, environmental sensing, state
estimation, automated design, control, and modular systems as separate open
areas. Its program included untethered self-reconfiguring flexible modules,
closed-loop tensegrity locomotion, and real-to-sim-to-real control. Workshop
programs are contextual evidence rather than archival technical records; claims
about individual systems must be traced to their papers.

Before manuscript submission, repeat and archive searches of IEEE Xplore,
Scopus, Web of Science, OpenAlex, and IROS workshop programs using combinations
of: `modular robot`, `self-reconfigurable`, `shape morphing`, `variable
footprint`, `navigation`, `path planning`, `kinodynamic`, and `SLAM`. Record
query dates, returned counts, screening decisions, and citation chaining. The
claim is an evidence-bounded literature inference, not proof that no other
system exists.

## Primary-source verification and code reuse (2026-09-11)

This bounded update inspected author repositories, the Freeform Robotics
[publication list](https://freeformrobotics.org/publications/), the Tu author
manuscript, upstream Nav2 source, and publisher-deposited Crossref metadata.
[Le metadata](https://api.crossref.org/works/10.3390/s18082585) confirms ROS
simulation and physical evaluation in the abstract.
[Cheng metadata](https://api.crossref.org/works/10.1109/ACCESS.2020.3006579)
confirms Cheng is first author; the previous attribution to Le was incorrect.
The Pankert author repository supplies the corrected citation. The
[Daudelin metadata](https://api.crossref.org/works/10.1126/scirobotics.aat4983)
was verified, but its full text was not inspected in this pass. Earlier rows
retained above were not all reverified; this is not a systematic corpus review.

Tu's author manuscript has a placeholder DOI; use the
[registered metadata](https://api.crossref.org/works/10.1177/02783649251360360)
for the final citation. Online publication was July 29, 2025; print publication
was March 2026. Its onboard configuration estimation combines magnetic sensing,
IMU, and wheel odometry. Do not confuse this work with the earlier FreeSN design
paper (ICRA 2022) or configuration-identification paper (T-RO 2023).

All external repositories below were **inspected remotely only**. None was
locally cloned, built, or tested during this review. Compatibility assessments
are implementation guidance, not validated integration results.

| Candidate | Source evidence | Reuse decision and limit |
|---|---|---|
| [swerve_steering](https://github.com/leggedrobotics/swerve_steering/tree/release) | README targets Ubuntu 20.04/ROS Noetic, with BSD-3-Clause licensing; source tree includes wheel rolling constraints, joint limits, MPC dynamics, steering/brake interfaces, robot models, and odometry. | Reference assembled steering kinematics and rolling constraints. Port selected components deliberately to ROS 2/Harmonic; fixed differential pods cannot become holonomic through command remapping. |
| [OCS2 ROS 2](https://github.com/leggedrobotics/ocs2/blob/ros2/installation.md) | The `ros2` branch documents Ubuntu 24.04/Jazzy and colcon. Pinocchio requires additional installation; MPC-Net and Raisim packages are excluded from that port. | Optional continuous-optimization backend after baseline mechanics are validated. Pin a tested revision before adopting it. |
| [smores_ros](https://github.com/MOD-ASL/smores_ros), [smores_reconfiguration](https://github.com/MOD-ASL/smores_reconfiguration) | Legacy ROS packages with sparse READMEs; the latter exposes a Python reconfiguration planner/node and a path catalog. | Inspect planner/executor separation and catalog semantics. They are not drop-in Jazzy dependencies. |
| [SimulationPlugins](https://github.com/MOD-ASL/SimulationPlugins) | README targets Gazebo API 4.0, describes docking/undocking plugins and a separate `GAZEBO_model` repository, and records historical Gazebo 1.9 issues. | Reference legacy module/sensor modeling and connection semantics; do not assume plugins or binaries work in Harmonic. |
| [Nav2 docking](https://github.com/open-navigation/opennav_docking) | Official README says the framework moved into Nav2 in June 2024. It provides staging, sensor-refined dock poses, contact checks, retries, feedback, and failure codes. | Use the installed Jazzy in-tree API and a modular-latch plugin. Charging-dock success and a simulator attachment event do not establish physical connector capture. |
| [MISO_connection](https://github.com/FreeformRobotics/MISO_connection) | README accepts initial/final adjacency matrices and returns a difference matrix; it explicitly restricts the model to Multiple In-degree Single Out-degree modules. | Candidate for compatible topology mapping only. It does not establish arbitrary typed-connector compatibility, docking trajectories, or collision feasibility. |

Upstream [Jazzy costmap source](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/src/costmap_2d_ros.cpp)
subscribes to `geometry_msgs/msg/Polygon` for footprint input and publishes
`PolygonStamped` output. Verify the installed package and graph before fixing
the manager's input publication; do not infer the input type from the published
footprint topic.

## What the novelty experiment must isolate

Use common mechanics, controller, sensor estimator, retry budget, and map
information across practical baselines. Include geometry-only coupled planning
to isolate the benefit of transition feasibility and sensing constraints, plus
sequential planning to isolate joint decision-making. An oracle using true
state or a complete map is a labeled upper bound, not a practical competitor.

Create cases where a short route has inadequate docking workspace or poor
connector visibility, while a longer route permits reliable reconfiguration.
Also include neutral cases where the added constraints should not help. Ablate
transition geometry, sensing constraints, and calibrated cost/risk separately.
Evaluate actual trajectory collision checks, sensor-based pose estimates,
capture/latch evidence, and recovery from partially completed transitions.
Ground-truth simulator pose may score estimation error but must not drive the
claimed sensing-aware autonomy. Simulator latching alone does not validate a
real connector or stacking mechanism.

The protocol below is the original design seed, not a completed experiment or
preregistered analysis. Its sample size requires power justification, and
success-only costs cannot by themselves support a mission-level superiority
claim when failure rates differ. Synthetic `edge_observations.csv` values are
scaffolding, not experimental evidence.

## Current research question and frozen methods

**Question:** Does joint route and morphology selection conditioned on 3D
transition feasibility and sensing uncertainty improve completion under a 300 s
deadline relative to route-first, geometry-only, and feasibility-only planning?

The confirmatory methods are `route_first_adaptation`, `geometry_coupled`,
`feasibility_coupled`, and `sensing_feasibility_coupled`. The target design uses
held-out reconfiguration-workspace, docking-observability, and combined
constraint families, includes neutral layouts, and treats layout as the
independent unit. Only the physically scoped `compact_diff` and
`narrow_tandem` morphologies enter this study. The frozen 432-trial schedule is
a resource target whose layout count remains subject to the disjoint pilot and
prospective power gate. The complete estimands and integrity rules live in
`statistical_analysis_plan.md`.
