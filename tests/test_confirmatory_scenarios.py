from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
from modular_robot_benchmarks.design import CONFIRMATORY_FAMILIES
from morphology_planner import HybridState, load_catalog, make_method_planner


def test_confirmatory_layouts_are_deterministic_and_manifested():
    for family in CONFIRMATORY_FAMILIES:
        first = make_confirmatory_scenario(family, 1, 12345)
        second = make_confirmatory_scenario(family, 1, 12345)
        assert first.grid.data == second.grid.data
        assert first.manifest() == second.manifest()
        assert first.layout_id == f"{family}-01"


def test_every_family_has_three_predeclared_neutral_controls():
    for family in CONFIRMATORY_FAMILIES:
        values = [make_confirmatory_scenario(family, index, 1000 + index)
                  for index in range(12)]
        assert sum(value.neutral_control for value in values) == 3
        assert all(not value.transition_obstacles and not value.observability_regions
                   for value in values if value.neutral_control)


def test_family_manipulations_are_isolated_and_sensing_is_location_dependent():
    catalog = load_catalog(
        "src/modular_robot_description/config/morphologies.yaml"
    ).supported_experiment_subset()
    transition = next(t for t in catalog.transitions if t.id == "compact_to_narrow")
    workspace = make_confirmatory_scenario("reconfiguration_workspace", 1, 8)
    sensing = make_confirmatory_scenario("docking_observability", 1, 8)
    combined = make_confirmatory_scenario("combined_constraints", 1, 8)
    assert workspace.transition_obstacles and not workspace.observability_regions
    assert sensing.observability_regions and not sensing.transition_obstacles
    assert combined.transition_obstacles and combined.observability_regions
    region = sensing.observability_regions[0]
    direct_xy = sensing.grid.world_to_cell(
        (region.min_x + region.max_x) / 2,
        (region.min_y + region.max_y) / 2,
    )
    direct = HybridState(*direct_xy, 0, "compact_diff")
    clear = HybridState(sensing.start[0], sensing.start[1] + 10, 0, "compact_diff")
    assert not sensing.sensing_for(transition, direct)["pod_0"].connector_visible
    assert sensing.sensing_for(transition, clear)["pod_0"].connector_visible


def test_generated_start_pose_clears_every_transition_obstacle():
    from itertools import product
    from math import cos, sin

    from modular_robot_benchmarks.evaluator_metrics import (
        CORE_COLLISION_BOX, POD_COLLISION_BOXES, _module_boxes, box_clearance,
    )

    # Compact pod model origins relative to the core, from modular_robot/model.sdf;
    # the core spawns at z 0.18 with pods 0.18 m below.
    pods = ((0.23, 0.20), (0.23, -0.20), (-0.23, 0.20), (-0.23, -0.20), (0.0, 0.20), (0.0, -0.20))
    for family in CONFIRMATORY_FAMILIES:
        for index in range(12):
            scenario = make_confirmatory_scenario(family, index, 2000 + index)
            sx, sy = scenario.grid.cell_center(*scenario.start)
            # Extremes of the declared spawn disturbance.
            for dx, dy, yaw in product((-0.015, 0.015), (-0.015, 0.015), (-0.035, 0.035)):
                x, y = sx + dx, sy + dy
                modules = [({"x": x, "y": y, "z": 0.18, "yaw": yaw}, (CORE_COLLISION_BOX,))]
                modules += [({"x": x + cos(yaw) * px - sin(yaw) * py,
                              "y": y + sin(yaw) * px + cos(yaw) * py,
                              "z": 0.0, "yaw": yaw}, POD_COLLISION_BOXES) for px, py in pods]
                for sample, boxes in modules:
                    for center, size, box_yaw in _module_boxes(sample, boxes):
                        for obstacle in scenario.transition_obstacles:
                            assert box_clearance(center, size, box_yaw,
                                                 obstacle.center, obstacle.size) > 0.05, (
                                scenario.layout_id, obstacle.name)


def test_layout_seeds_change_saved_geometry_parameters():
    first = make_confirmatory_scenario("combined_constraints", 1, 1)
    second = make_confirmatory_scenario("combined_constraints", 1, 2)
    assert first.parameters != second.parameters


def _transition_site(method, scenario, catalog):
    planner = make_method_planner(
        method, catalog, scenario.grid, heading_bins=4,
        environment=scenario.transition_obstacles,
        sensing_provider=scenario.sensing_for,
    )
    plan = planner.plan(
        HybridState(*scenario.start, 0, "compact_diff"), scenario.goal, 2.5)
    segment = next(value for value in plan.segments
                   if value.transition_id == "compact_to_narrow")
    return (segment.source.x, segment.source.y), planner


def test_golden_layouts_separate_3d_and_sensing_ablation_decisions():
    catalog = load_catalog(
        "src/modular_robot_description/config/morphologies.yaml"
    ).supported_experiment_subset()
    workspace = make_confirmatory_scenario("reconfiguration_workspace", 1, 8)
    geometry_site, geometry = _transition_site("geometry_coupled", workspace, catalog)
    feasible_site, feasible = _transition_site("feasibility_coupled", workspace, catalog)
    assert geometry_site != feasible_site
    assert any("environment_collision" in reason
               for decision in feasible.transition_policy.decisions
               for reason in decision.reasons)

    observability = make_confirmatory_scenario("docking_observability", 1, 8)
    feasible_site, _ = _transition_site("feasibility_coupled", observability, catalog)
    sensing_site, sensing = _transition_site(
        "sensing_feasibility_coupled", observability, catalog)
    assert feasible_site != sensing_site
    assert any("connector_not_visible" in reason
               for decision in sensing.transition_policy.decisions
               for reason in decision.reasons)

    combined = make_confirmatory_scenario("combined_constraints", 1, 8)
    feasible_site, _ = _transition_site("feasibility_coupled", combined, catalog)
    sensing_site, _ = _transition_site(
        "sensing_feasibility_coupled", combined, catalog)
    assert feasible_site != sensing_site
