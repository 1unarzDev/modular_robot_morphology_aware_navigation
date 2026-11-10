# Frozen Nav2 configurations for the ExperimentChoice doorway family

Exploratory. Not evidence for the morphology study and never pooled with it.
Files: `src/modular_robot_bringup/config/expchoice/`. Hashes are bound in
`modular_robot_benchmarks/doorway_design.py` (`NAV2_CONFIGS`), in every design
that names a file, and checked again when a trial is prepared.

| Morphology | Arm | Method id | File | sha256 |
|---|---|---|---|---|
| compact_diff | A | `rpp_rotate_navfn` | `compact_diff_A_rpp_rotate_navfn.yaml` | `50d697d5a0159da1b3e44419134e2bb2068f5e21a9032ee84f3e7b1d377908d1` |
| compact_diff | B | `mppi_reverse_navfn` | `compact_diff_B_mppi_reverse_navfn.yaml` | `9ebe0ef54e798be43adfae4a771bf62df4d5d8752ed870d5192e5cda6e1de512` |
| compact_diff | C | `rpp_reverse_smac_rs` | `compact_diff_C_rpp_reverse_smac_rs.yaml` | `ebd94d2d681964a8387242a837a87e8364890dc3518b9abe87c902498ae94617` |
| narrow_tandem | A | `rpp_rotate_navfn` | `narrow_tandem_A_rpp_rotate_navfn.yaml` | `8eabb941e04acaf0596a9afd927bbaefb1fffde7f717cacd632116e46432f80c` |
| narrow_tandem | B | `mppi_reverse_navfn` | `narrow_tandem_B_mppi_reverse_navfn.yaml` | `4dcb4a6e09c3b8178ccc8323a90fcda20028dbfe47fd67399e98080a5804da55` |
| narrow_tandem | C | `rpp_reverse_smac_rs` | `narrow_tandem_C_rpp_reverse_smac_rs.yaml` | `3749c8150b8a0dedd3b52321a2c0084a847aa5a4b8399318cef2a99c41ab0966` |

The pair for the first study is **A against B**. C is a second reverse-capable
configuration, frozen but not scheduled (see "Why B and not C").
narrow_tandem files are frozen ahead of a way to spawn that morphology; no
narrow_tandem trial can run today.

## What every arm shares

Copied from `config/nav2.yaml` at 00ac590 and identical across arms (a test
compares them key by key): both costmaps (footprint, obstacle and inflation
layers, inflation radius 0.45 m, cost scaling 4.0, local 0.025 m), the goal
checker (xy 0.12 m, yaw free, not stateful), the progress checker (0.15 m in
20 s), the velocity smoother (1.2 m/s^2, 2.0 rad/s^2, reverse to -0.4 m/s), the
collision monitor, the behaviour server (spin, backup, wait), bt_navigator
(stock default tree), AMCL and the map server (`localization.yaml`).

Changes from `nav2.yaml` that apply to every arm:

| Change | Reason |
|---|---|
| Controller block renamed `FollowPath`; the five other controllers removed | The stock behaviour tree asks for `FollowPath`. The other studies address controllers by name through the hybrid navigator, which this family does not use. |
| Keepout costmap filters removed | This family serves no keepout mask; the filter would wait for one. |
| `use_rotate_to_heading`, `allow_reversing` written out in the pure pursuit arms | Explicit; for A these are Nav2's defaults and the repository's behaviour. |

## A against B (compact_diff; narrow_tandem values in brackets)

Only `controller_server.FollowPath` differs. Planner, costmaps, checkers,
smoother, monitor, behaviours and tree are the same file content.

| Parameter (A -> B) | Reason | Confound? |
|---|---|---|
| `plugin`: RegulatedPurePursuit -> MPPIController, `motion_model: DiffDrive` | The contrast under test: A turns in place and only drives forward; B can turn in place and reverse. MPPI DiffDrive with `vx_min < 0` is the controller stock Nav2 Jazzy ships by default. | **Yes, the main one.** The whole control law changes, not only the ability to reverse: sampling optimiser against a geometric follower, cost-aware steering against stop-on-collision, different behaviour at the goal, different CPU load. A B win in a tight bay cannot be attributed to reversing alone; the record's stretch and reverse-travel counts say whether B reversed at all. |
| `vx_max` 0.35 [0.18] = A's `desired_linear_vel`; `vx_min` -0.35 [-0.18] | Same top speed; reverse as fast as forward and inside the smoother's -0.4 m/s. | Reverse speed is B's only. |
| `wz_max` 0.6 [0.4] = A's `rotate_to_heading_angular_vel` | Same in-place turn rate. | A's turn rate while following a curve is not capped at this value (only by the smoother's 1.8 rad/s), so B is the more restricted on curves. |
| `ax_max` 1.2, `ax_min` -1.2, `az_max` 2.0 | The velocity smoother's limits, which bound A as well. | A's own `max_angular_accel` (1.0 [30.0]) limits only its rotate-to-heading ramp; not matched. |
| `transform_tolerance` 0.2 | Same as A. | No. |
| `CostCritic.consider_footprint: true` (stock: false) | The footprint is a rectangle; stock MPPI scores a circle of the inscribed radius, which is wrong in a bay a few centimetres longer than the robot. A's collision check already uses the footprint. | Makes B slower to compute. |
| `visualize: false` (stock: true) | No viewer; saves controller time. | No. |
| All other MPPI values (56 steps of 0.05 s, 2000 samples, noise, temperature, the eight critics and their weights) | Nav2 Jazzy's stock `nav2_params.yaml`, unchanged, so nothing was tuned toward a result. | Stock values assume 0.5 m/s; `prune_distance` and the critics' thresholds were not rescaled to 0.35 m/s. B's noise is resampled every cycle and is not seeded: B has run-to-run variance A does not. |
| `PreferForwardCritic` (weight 5.0), part of the stock set | Kept as shipped. | B may reverse but is penalised for it; "reverse-capable", not "reverse-neutral". |
| A's lookahead, approach, regulation and rotate parameters | No counterpart in MPPI. | Part of the main confound. |

## A against C

| Parameter (A -> C) | Reason | Confound? |
|---|---|---|
| `FollowPath.use_rotate_to_heading` true -> false | Nav2 refuses `allow_reversing` together with rotate-to-heading. | **Yes.** C loses in-place rotation, so it is a car-like follower: it differs from A in two capabilities at once. |
| `FollowPath.allow_reversing` false -> true | The contrast. | No. |
| `GridBased.plugin` NavFn A* -> SmacPlannerHybrid, `motion_model_for_search: REEDS_SHEPP` | Pure pursuit only reverses along a path that has cusps; NavFn never produces one. This is the pairing Nav2's documentation gives for reversing. | **Yes.** NavFn plans for a point on inflated costs; Smac checks the rectangular footprint at 72 headings and respects a turning radius. C can refuse a goal A's planner accepts, and takes longer to plan. |
| `minimum_turning_radius` 0.4 | Nav2's default; nothing smaller was justified without a run. | A constraint only C has. |
| `downsample_costmap` true, `downsampling_factor` 5 | The global costmap inherits the map's 0.01 m cells; Smac's lookup tables and search are sized for about 0.05 m. | A plans at 0.01 m, C at 0.05 m. Exact for the first batch (every edge on the 5 cm lattice); otherwise conservative by under 5 cm per face. |
| `tolerance` 0.25, `allow_unknown` true | Same as A's planner. | No. |
| Remaining Smac values | Nav2 Jazzy defaults. | Goal heading: Smac plans to the goal pose's yaw (0, along the door axis); A ignores it. |

## Why B and not C for the first study

The hypothesis is about a robot that can turn in place: without reversing it
needs bay depth 2 rho to turn; with reversing it can shimmy in less. B keeps
in-place turning and adds reversing. C trades in-place turning for reversing,
so in a bay it needs many Reeds-Shepp strokes even when A could simply pivot,
and a 0.05 m lattice with a 0.4 m turning radius is unlikely to find a shimmy
inside centimetres of slack. C is the cleaner change in parameters and the
worse match to the mechanism.

## Confounds common to both comparisons

- **Recoveries reverse.** The stock tree's recovery sequence is clear costmaps,
  spin 1.57 rad, wait 5 s, back up 0.30 m at 0.15 m/s. Arm A therefore can move
  backwards, through the behaviour server, after its controller fails. "A does
  not reverse" is a statement about its controller. Each record carries
  `recovery_actions` and `controller_diagnostics.expchoice.motion`.
- **Replanning at 1 Hz** by the stock tree, in every arm.
- **The tree is not hashed.** It is the image's `nav2_bt_navigator` default;
  the record carries the image digest.

## Plugin availability in `lunarzdev/astro:core`

Checked without starting a container.

| Plugin | Evidence | Status |
|---|---|---|
| RegulatedPurePursuitController, NavfnPlanner, SimpleGoalChecker, SimpleProgressChecker, Spin/BackUp/Wait, bt_navigator `navigate_to_pose`, AMCL, map_server, collision monitor, velocity smoother | Created and activated in an earlier campaign's `launch.log` from this image (2026-11-09) | Verified loaded |
| MPPIController, `motion_model: DiffDrive` | Same log: four MPPI controllers created, configured and activated, two of them DiffDrive | Verified loaded. **Not verified:** that this image's version accepts `ax_max`/`ax_min`/`az_max` (undeclared names are ignored, not fatal) and that the stock critic set is complete in it |
| SmacPlannerHybrid | `nav2_smac_planner` is in the image's package list (`astro_dock/.devcontainer/packages.txt`) and a dependency of `nav2_bringup` via `navigation2` | **Unverified at run time**: never instantiated in any log on this host |
| Stock behaviour tree with `FollowPath` / `GridBased` defaults | Upstream `jazzy` branch (1.3.14); the image's exact Nav2 version is not recorded anywhere readable | **Unverified** for the installed version |

Parameter names were checked against the upstream `jazzy` branch sources
(1.3.14, commit 94d1c2a): pure pursuit `parameter_handler.cpp`, Smac
`smac_planner_hybrid.cpp` and `types.hpp`, NavFn `navfn_planner.cpp`, and the
MPPI block of `nav2_bringup/params/nav2_params.yaml`. The image was built from
apt on 2026-11-05 and may carry an older 1.3.x.
