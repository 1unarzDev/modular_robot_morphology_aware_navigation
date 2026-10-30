from pathlib import Path

import pytest

from morphology_planner import (
    HybridState, MorphologyAStar, OccupancyGrid, RouteFirstAdaptationPlanner,
    load_catalog,
)
from morphology_planner.grid import OCCUPIED
from morphology_planner.grid import UNKNOWN
from morphology_planner.methods import make_method_planner
from morphology_planner.planner import NoPathError
from morphology_planner.transition_validation import Box3


CATALOG = Path(__file__).parents[1] / "src/modular_robot_description/config/morphologies.yaml"


def doorway_grid():
    # Leave a full reconfiguration workspace on both sides of the wall. The
    # 1.76 m narrow body and 1.0 m transition sweep must not be clipped by the
    # artificial map boundary at the start or goal.
    grid = OccupancyGrid(55, 25, 0.1)
    for y in range(grid.height):
        if not 10 <= y <= 16:
            grid.set_value(20, y, OCCUPIED)
    return grid


def test_hybrid_plan_reconfigures_for_narrow_doorway():
    planner = MorphologyAStar(
        load_catalog(CATALOG).supported_experiment_subset(), doorway_grid(), heading_bins=4)
    plan = planner.plan(HybridState(7, 13, 0, "compact_diff"), (42, 13), epsilon=2.5)
    transitions = [s.transition_id for s in plan.segments if s.kind == "reconfigure"]
    assert "compact_to_narrow" in transitions
    assert any(state.x == 20 and state.morphology == "narrow_tandem" for state in plan.states)


def test_fixed_compact_robot_cannot_cross_narrow_doorway():
    catalog = load_catalog(CATALOG)
    catalog = type(catalog)(catalog.morphologies, (), catalog.objective)
    planner = MorphologyAStar(catalog, doorway_grid(), heading_bins=4)
    with pytest.raises(NoPathError):
        planner.plan(HybridState(7, 13, 0, "compact_diff"), (42, 13), epsilon=2.5)


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


def _shelf_ahead(z_center: float):
    # Spans the compact robot's left pod track just beyond its front edge.
    return (Box3("shelf", (1.8, 2.25, z_center), (0.6, 0.16, 0.08)),)


@pytest.mark.parametrize("method", [
    "route_first_adaptation", "geometry_coupled",
    "feasibility_coupled", "sensing_feasibility_coupled",
])
def test_raised_obstacle_below_robot_height_blocks_driving_for_every_method(method):
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    source = HybridState(10, 20, 0, "compact_diff")
    forward = HybridState(11, 20, 0, "compact_diff")
    for z_center, blocked in ((0.14, True), (0.50, False)):
        wrapper = make_method_planner(
            method, catalog, OccupancyGrid(40, 40, 0.1), heading_bins=4,
            environment=_shelf_ahead(z_center))
        hybrid = getattr(wrapper.planner, "hybrid", wrapper.planner)
        assert hybrid._state_is_free(source)
        assert hybrid._traversal_is_free(source, forward) is not blocked


def test_raised_obstacle_leaves_transition_disk_check_to_the_policy():
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    planner = MorphologyAStar(catalog, OccupancyGrid(40, 40, 0.1), heading_bins=4,
                              drive_obstacles=_shelf_ahead(0.14))
    compact_to_narrow = next(t for t in catalog.transitions if t.id == "compact_to_narrow")
    # Only driving sees the shelf; the 2D transition check is unchanged.
    assert planner._validate_transition(compact_to_narrow, HybridState(20, 20, 0, "compact_diff"))


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
    assert joint.plan(start, (42, 13)).segments
    sequential = RouteFirstAdaptationPlanner(
        catalog, grid, heading_bins=4, transition_validator=validator)
    with pytest.raises(NoPathError, match="fixed spatial route"):
        sequential.plan(start, (42, 13))


def test_indexed_footprint_check_matches_a_cell_by_cell_scan():
    """The blocked-cell index is an optimization; it must never change an answer."""
    from math import cos, sin
    from random import Random

    from morphology_planner.grid import (
        OCCUPIED, UNKNOWN, OccupancyGrid, _polygon_intersects_cell)

    def reference(grid, wx, wy, yaw, footprint, unknown_is_occupied):
        c, s = cos(yaw), sin(yaw)
        polygon = tuple((wx + c * x - s * y, wy + s * x + c * y) for x, y in footprint)
        cmin_x, cmin_y = grid.world_to_cell(min(x for x, _ in polygon), min(y for _, y in polygon))
        cmax_x, cmax_y = grid.world_to_cell(max(x for x, _ in polygon), max(y for _, y in polygon))
        for gy in range(cmin_y - 1, cmax_y + 2):
            for gx in range(cmin_x - 1, cmax_x + 2):
                value = grid.value(gx, gy)
                if value >= OCCUPIED or (unknown_is_occupied and value == UNKNOWN):
                    if _polygon_intersects_cell(polygon, *grid.cell_center(gx, gy), grid.resolution):
                        return False
        return True

    rng = Random(7)
    grid = OccupancyGrid(40, 30, 0.1)
    for _ in range(120):
        grid.set_value(rng.randrange(40), rng.randrange(30), rng.choice((OCCUPIED, UNKNOWN)))
    footprint = ((0.88, 0.33), (-0.88, 0.33), (-0.88, -0.33), (0.88, -0.33))
    for trial in range(600):
        if trial == 300:   # the index must follow a change to the grid
            for _ in range(60):
                grid.set_value(rng.randrange(40), rng.randrange(30), OCCUPIED)
        pose = (rng.uniform(-0.5, 4.5), rng.uniform(-0.5, 3.5), rng.uniform(-3.2, 3.2))
        for unknown in (False, True):
            assert grid.footprint_is_free(*pose, footprint, unknown) == reference(
                grid, *pose, footprint, unknown)


def test_polygon_rectangle_test_is_unchanged_by_its_bounding_box_rejection():
    from math import cos, sin
    from random import Random

    from morphology_planner.grid import _segments_intersect, _point_in_polygon, polygon_intersects_rectangle

    def reference(polygon, left, bottom, right, top):
        corners = ((left, bottom), (right, bottom), (right, top), (left, top))
        if any(_point_in_polygon(x, y, polygon) for x, y in corners):
            return True
        if any(left <= x <= right and bottom <= y <= top for x, y in polygon):
            return True
        return any(_segments_intersect(*a, *b)
                   for a in zip(polygon, polygon[1:] + polygon[:1])
                   for b in zip(corners, corners[1:] + corners[:1]))

    rng = Random(11)
    for _ in range(3000):
        cx, cy, yaw = rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-3.2, 3.2)
        hx, hy = rng.uniform(0.05, 0.9), rng.uniform(0.05, 0.4)
        c, s = cos(yaw), sin(yaw)
        polygon = tuple((cx + c * x - s * y, cy + s * x + c * y)
                        for x, y in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)))
        left, bottom = rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5)
        size = rng.choice((0.1, 0.3))
        rect = (left, bottom, left + size, bottom + size)
        assert polygon_intersects_rectangle(polygon, *rect) == reference(polygon, *rect)


def test_a_start_snapped_into_collision_is_repaired_to_the_nearest_free_state():
    """Pilot: a 1.76 m body at ~0.45 rad snaps to the 45-degree bin beside a wall."""
    import pytest

    from morphology_planner import HybridState, load_catalog, make_method_planner
    from morphology_planner.grid import OCCUPIED, OccupancyGrid
    from morphology_planner.planner import NoPathError

    catalog = load_catalog("src/modular_robot_description/config/morphologies.yaml"
                           ).supported_experiment_subset()
    grid = OccupancyGrid(60, 35, 0.1)
    for x in range(60):
        grid.set_value(x, 0, OCCUPIED)
        grid.set_value(x, 34, OCCUPIED)
    for y in range(35):
        grid.set_value(0, y, OCCUPIED)
        grid.set_value(59, y, OCCUPIED)
    planner = make_method_planner("geometry_coupled", catalog, grid, heading_bins=8).planner
    snapped = HybridState(30, 29, 1, "narrow_tandem")   # 45 deg: nose into the top wall
    assert not planner._state_is_free(snapped)
    repaired = planner.repaired_start(snapped)
    assert planner._state_is_free(repaired)
    assert abs(repaired.x - 30) <= 2 and abs(repaired.y - 29) <= 2
    free = HybridState(30, 17, 0, "narrow_tandem")
    assert planner.repaired_start(free) == free
    walled_in = OccupancyGrid(5, 5, 0.1, data=[OCCUPIED] * 25)
    trapped = make_method_planner("geometry_coupled", catalog, walled_in, heading_bins=8).planner
    with pytest.raises(NoPathError, match="start state is in collision"):
        trapped.repaired_start(HybridState(2, 2, 0, "narrow_tandem"))
