from pathlib import Path

import pytest

from morphology_planner import HybridState, MorphologyAStar, OccupancyGrid, load_catalog
from morphology_planner.grid import OCCUPIED
from morphology_planner.grid import UNKNOWN
from morphology_planner.planner import NoPathError


CATALOG = Path(__file__).parents[1] / "src/modular_robot_description/config/morphologies.yaml"


def doorway_grid():
    grid = OccupancyGrid(40, 25, 0.1)
    for y in range(grid.height):
        if not 11 <= y <= 14:
            grid.set_value(20, y, OCCUPIED)
    return grid


def test_hybrid_plan_reconfigures_for_narrow_doorway():
    planner = MorphologyAStar(load_catalog(CATALOG), doorway_grid(), heading_bins=4)
    plan = planner.plan(HybridState(7, 13, 0, "compact_diff"), (32, 13), epsilon=2.5)
    transitions = [s.transition_id for s in plan.segments if s.kind == "reconfigure"]
    assert "compact_to_narrow" in transitions
    assert plan.states[-1].morphology == "narrow_tandem"


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
