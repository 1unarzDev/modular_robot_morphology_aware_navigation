# Morphology-Aware Navigation for Modular Robots: Evidence and Study Gap

## Conclusion

The novelty claim cannot be that no planner searches over position and
morphology. hTetro work already selects discrete shapes during coverage and
jointly optimizes actions, paths, and configurations. Variable-footprint mobile
bases have also coupled navigation and reconfiguration with model-predictive
control. The defensible gap is a reusable full-ROS navigation system for a typed
module graph that accounts for morphology-dependent footprint, kinodynamics,
sensing, and physically feasible topology changes while replanning from online
maps.

## Closest prior art

| Area | Result | Remaining limitation for this study | Primary source |
|---|---|---|---|
| Joint path and morphology | Modified A* selects among hTetro's four morphologies to cover narrow regions. | Platform-specific planar coverage with a small discrete shape set. | Le et al., *Sensors* 2018, [doi:10.3390/s18082585](https://doi.org/10.3390/s18082585) |
| Joint path and morphology | A genetic algorithm jointly optimizes hTetro actions, route, and configurations in simulation and hardware. | No general typed-module ROS navigation architecture or online kinodynamic replanning. | Le et al., *IEEE Access* 2020, [doi:10.1109/ACCESS.2020.3006579](https://doi.org/10.1109/ACCESS.2020.3006579) |
| Coverage and reconfiguration | Graph search and dynamic programming combine coverage with hTetro morphology shifts. | Complete-coverage objective on one platform. | Le et al., *IEEE Access* 2019, [doi:10.1109/ACCESS.2019.2928467](https://doi.org/10.1109/ACCESS.2019.2928467) |
| Continuous footprint control | MPC jointly controls navigation, footprint reconfiguration, and manipulation. | Continuously variable, fixed-topology mobile base. | Gawel et al., *RA-L* 2022, [doi:10.1109/LRA.2022.3189166](https://doi.org/10.1109/LRA.2022.3189166) |
| Restricted-space platform | MIRRAX changes Mecanum-wheel geometry and demonstrates configuration-dependent controllability. | Platform/control contribution rather than route-dependent topology search. | West et al. 2022, [arXiv:2203.00337](https://arxiv.org/abs/2203.00337) |
| Mission-level selection | SMORES-EP maps task requirements to a library of configurations and behaviors; VSPARC transfers Unity designs to hardware. | Symbolic regions and stored behaviors rather than geometric/kinodynamic navigation. | Jing et al., RSS 2016, [doi:10.15607/RSS.2016.XII.025](https://doi.org/10.15607/RSS.2016.XII.025) |
| Morphology-general control | A graph-structured policy generalizes locomotion across simulated and physical module graphs. | Controls a supplied morphology; it does not decide where to reconfigure. | Whitman et al., *T-RO* 2023, [doi:10.1109/TRO.2023.3284362](https://doi.org/10.1109/TRO.2023.3284362) |
| Modular motion control | A QP framework adapts to changing kinematics and coupled goals on CKBot and SMORES-EP. | Local motion control without route-dependent topology selection. | Liu et al., IRC 2020, [doi:10.1109/IRC.2020.00008](https://doi.org/10.1109/IRC.2020.00008) |
| Reconfiguration planning | A two-layer planner generates module motions along a supplied trajectory. | Desired route/configuration is supplied rather than jointly selected. | Yoshida et al., *IJRR* 2002, [doi:10.1177/0278364902021010835](https://doi.org/10.1177/0278364902021010835) |
| Distributed reconfiguration | SMORES modules transform to a specified target configuration. | Target morphology is an input rather than a navigation decision. | Liu et al., *RA-L* 2019, [doi:10.1109/LRA.2019.2930432](https://doi.org/10.1109/LRA.2019.2930432) |
| Flexible connectors | FreeBOT modules move independently and attach magnetically at arbitrary spherical locations. | Connection hardware without an integrated morphology-aware navigation stack. | IROS 2020, [doi:10.1109/IROS45743.2020.9341129](https://doi.org/10.1109/IROS45743.2020.9341129) |

## IROS workshop evidence

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

## Research question and hypotheses

**Question:** Does coupled pose–morphology planning improve completion and
time–energy–risk cost over fixed and sequential planning in partially mapped
indoor environments whose regions favor different footprints and kinematics?

- H1: coupled planning increases mission completion on tasks requiring at least
  one morphology change.
- H2: among successful runs, coupled planning reduces the preregistered
  time–energy–risk objective.
- H3: learned traversal and docking models reduce predicted-versus-observed cost
  error and unnecessary reconfiguration relative to analytic models.
- H4: lazy transition validation reduces planning latency without changing the
  feasible solution set.

## Experimental protocol

Use paired trials on identical layouts and physics seeds across five scenario
families: wide-versus-narrow route choice, open–narrow–open, tight compound
turns, long corridor followed by maneuvering space, and an initially occluded
shortcut or blockage. Evaluate at least 30 independent layouts per family and
five physics/noise seeds per layout.

Compare best fixed morphology with hindsight, preselected fixed morphology,
route-first local adaptation, one complete route per morphology, coupled search
with analytic costs, and coupled search with learned costs. Report completion,
the preregistered objective, time, energy, planning latency, clearance,
collisions, docking attempts, transition duration, path regret, localization
error, and model calibration. Analyze paired effects with layout as a random
effect, publish confidence intervals, and retain failed trials.

