from pathlib import Path

import pytest

from modular_robot_benchmarks.route_first_retry import RouteFirstRetryPlanner
from morphology_planner import HybridState, OccupancyGrid, RouteFirstAdaptationPlanner, load_catalog
from morphology_planner.grid import OCCUPIED
from morphology_planner.planner import NoPathError
from morphology_planner.route_first import SpatialState


CATALOG = Path(__file__).parents[1] / "src/modular_robot_description/config/morphologies.yaml"


def _room():
    grid = OccupancyGrid(70, 25, 0.1)
    for y in range(grid.height):
        if not 10 <= y <= 16:
            grid.set_value(30, y, OCCUPIED)
    return grid


def test_one_route_retry_fails_exactly_as_route_first():
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    start = HybridState(7, 13, 0, "compact_diff")
    validator = lambda transition, state: state.heading == 1
    with pytest.raises(NoPathError, match="fixed spatial route"):
        RouteFirstAdaptationPlanner(
            catalog, _room(), heading_bins=4,
            transition_validator=validator).plan(start, (55, 13))
    retry = RouteFirstRetryPlanner(
        catalog, _room(), heading_bins=4, transition_validator=validator,
        max_routes=1)
    with pytest.raises(NoPathError, match="none of 1 spatial routes"):
        retry.plan(start, (55, 13))
    assert [attempt["adapted"] for attempt in retry.attempts] == [False]


def test_a_failed_route_makes_its_neighbourhood_dearer_and_only_that():
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    planner = RouteFirstRetryPlanner(
        catalog, _room(), heading_bins=4, penalty_radius=1, penalty_weight=2.0)
    near, far = SpatialState(10, 13, 0), SpatialState(10, 3, 0)
    before = {state: planner._spatial_neighbors(state) for state in (near, far)}
    planner._penalize(tuple((x, 13) for x in range(5, 25)))
    after = {state: planner._spatial_neighbors(state) for state in (near, far)}
    assert after[far] == before[far]
    for target, cost in after[near].items():
        if abs(target.y - 13) <= 1:
            assert cost == pytest.approx(3.0 * before[near][target])
