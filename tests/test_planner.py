from pathlib import Path

import pytest

from morphology_planner import (
    HybridState, MorphologyAStar, OccupancyGrid, RouteFirstAdaptationPlanner,
    load_catalog,
)
from morphology_planner.grid import OCCUPIED
from morphology_planner.grid import UNKNOWN
from morphology_planner.planner import NoPathError


CATALOG = Path(__file__).parents[1] / "src/modular_robot_description/config/morphologies.yaml"


def doorway_grid():
    grid = OccupancyGrid(40, 25, 0.1)
    for y in range(grid.height):
        if not 10 <= y <= 16:
            grid.set_value(20, y, OCCUPIED)
    return grid


def test_hybrid_plan_reconfigures_for_narrow_doorway():
    planner = MorphologyAStar(
        load_catalog(CATALOG).supported_experiment_subset(), doorway_grid(), heading_bins=4)
    plan = planner.plan(HybridState(7, 13, 0, "compact_diff"), (32, 13), epsilon=2.5)
    transitions = [s.transition_id for s in plan.segments if s.kind == "reconfigure"]
    assert "compact_to_narrow" in transitions
    assert any(state.x == 20 and state.morphology == "narrow_tandem" for state in plan.states)


def test_fixed_compact_robot_cannot_cross_narrow_doorway():
    catalog = load_catalog(CATALOG)
    catalog = type(catalog)(catalog.morphologies, (), catalog.objective)
    planner = MorphologyAStar(catalog, doorway_grid(), heading_bins=4)
    with pytest.raises(NoPathError):
        planner.plan(HybridState(7, 13, 0, "compact_diff"), (32, 13), epsilon=2.5)


def test_reconfiguration_needs_swept_volume_clearance():
    grid = doorway_grid()
    planner = MorphologyAStar(load_catalog(CATALOG), grid, heading_bins=4)
    compact_to_narrow = next(t for t in planner.catalog.transitions if t.id == "compact_to_narrow")
    near_wall = HybridState(18, 13, 0, "compact_diff")
    assert not planner.transition_validator(compact_to_narrow, near_wall)


def test_unknown_space_adds_risk_without_becoming_an_obstacle():
    catalog = load_catalog(CATALOG)
    free = OccupancyGrid(20, 20, 0.1)
    unknown = OccupancyGrid(20, 20, 0.1)
    for x in range(8, 15):
        for y in range(7, 14):
            unknown.set_value(x, y, UNKNOWN)
    start = HybridState(5, 10, 0, "compact_diff")
    free_plan = MorphologyAStar(catalog, free, heading_bins=4).plan(start, (15, 10))
    unknown_plan = MorphologyAStar(catalog, unknown, heading_bins=4).plan(start, (15, 10))
    assert unknown_plan.total_cost > free_plan.total_cost


def test_grid_coarsening_preserves_obstacles_and_unknown_regions():
    grid = OccupancyGrid(4, 4, 0.05, data=[0] * 16, revision=7)
    grid.data[0] = OCCUPIED
    for y in range(2, 4):
        for x in range(2, 4):
            grid.data[grid.index(x, y)] = UNKNOWN
    coarse = grid.coarsen(0.1)
    assert (coarse.width, coarse.height, coarse.resolution) == (2, 2, 0.1)
    assert coarse.value(0, 0) == OCCUPIED
    assert coarse.value(1, 1) == UNKNOWN
    assert coarse.revision == 7


def test_grid_padding_preserves_map_and_marks_unmapped_goal_unknown():
    grid = OccupancyGrid(4, 3, 0.1, origin_x=1.0, origin_y=2.0,
                         data=list(range(12)), revision=9)
    padded = grid.padded_to_include(((0.8, 2.1), (1.7, 2.1)), margin=0.1)
    assert padded.revision == 9
    assert padded.value(*padded.world_to_cell(1.05, 2.05)) == 0
    assert padded.value(*padded.world_to_cell(0.8, 2.1)) == UNKNOWN
    assert padded.value(*padded.world_to_cell(1.7, 2.1)) == UNKNOWN


def test_motion_primitive_checks_swept_footprint_between_endpoints():
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    grid = OccupancyGrid(50, 50, 0.05)
    # Both endpoint headings fit; the intermediate rotation clips this cell.
    grid.set_value(18, 11, OCCUPIED)
    planner = MorphologyAStar(catalog, grid, heading_bins=4)
    source = HybridState(20, 20, 0, "compact_diff")
    target = HybridState(20, 20, 1, "compact_diff")
    assert planner._state_is_free(source)
    assert planner._state_is_free(target)
    assert not planner._traversal_is_free(source, target)


def test_footprint_rejects_occupied_cell_corner_overlap():
    grid = OccupancyGrid(10, 10, 0.1)
    grid.set_value(5, 5, OCCUPIED)
    # The polygon clips the cell's lower-left corner without containing its center.
    footprint = ((-0.04, -0.04), (0.04, -0.04), (0.04, 0.04), (-0.04, 0.04))
    assert not grid.footprint_is_free(0.49, 0.49, 0.0, footprint)


def test_route_first_baseline_does_not_revise_route_for_transition_feasibility():
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    grid = doorway_grid()
    start = HybridState(7, 13, 0, "compact_diff")
    # The transition is feasible only after an in-place staging rotation. Joint
    # search can add it, while the frozen route contains only heading-zero poses.
    validator = lambda transition, state: state.heading == 1
    joint = MorphologyAStar(catalog, grid, heading_bins=4,
                            transition_validator=validator)
    assert joint.plan(start, (32, 13)).segments
    sequential = RouteFirstAdaptationPlanner(
        catalog, grid, heading_bins=4, transition_validator=validator)
    with pytest.raises(NoPathError, match="fixed spatial route"):
        sequential.plan(start, (32, 13))
