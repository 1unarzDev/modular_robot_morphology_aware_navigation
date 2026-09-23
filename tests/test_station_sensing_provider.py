"""The planner's prediction of station observability at a candidate site.

ADR 0006: precision docking depends on an external workspace fiducial station,
so a site is well observed only where the station can see where the pods will
be docked. The planner predicts that from priors alone.
"""

from pathlib import Path

from morphology_planner import (
    FiducialStation, HybridState, OccupancyGrid, load_catalog,
    station_sensing_provider,
)
from morphology_planner.observability import (
    STATION_UNOBSERVED_TRACE, precise_trace,
)
from morphology_planner.transition_validation import Box3

CATALOG = Path(__file__).parents[1] / "src/modular_robot_description/config/morphologies.yaml"
GATE = 0.015
HEADING_BINS = 8


def _setup():
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    transition = next(t for t in catalog.transitions if t.id == "compact_to_narrow")
    grid = OccupancyGrid(60, 35, 0.1)
    return catalog, transition, grid


def _site(grid, x=20, y=17):
    return HybridState(x, y, 0, "compact_diff")


def test_a_site_the_station_can_see_is_observed_and_precise():
    catalog, transition, grid = _setup()
    wx, wy = grid.cell_center(20, 17)
    station = FiducialStation("east", (wx, wy + 3.0, 1.6))
    provider = station_sensing_provider(
        catalog, (station,), (), grid, HEADING_BINS)
    sensing = provider(transition, _site(grid))
    assert set(sensing) == set(transition.moved_pods)
    assert all(state.connector_visible for state in sensing.values())
    assert all(state.covariance_trace < GATE for state in sensing.values())


def test_no_declared_station_leaves_every_pod_unobserved():
    """A world that declares no station must not be planned as station-gated;
    the planner node only installs this provider when stations exist."""
    catalog, transition, grid = _setup()
    provider = station_sensing_provider(catalog, (), (), grid, HEADING_BINS)
    sensing = provider(transition, _site(grid))
    assert not any(state.connector_visible for state in sensing.values())
    assert all(state.covariance_trace == STATION_UNOBSERVED_TRACE
               for state in sensing.values())


def test_an_occluder_between_station_and_site_makes_it_unobserved():
    catalog, transition, grid = _setup()
    wx, wy = grid.cell_center(20, 17)
    station = FiducialStation("east", (wx, wy + 3.0, 1.6))
    # A tall screen across the station's view of the whole site.
    screen = Box3("screen", (wx, wy + 1.5, 1.0), (4.0, 0.1, 2.0))
    provider = station_sensing_provider(
        catalog, (station,), (screen,), grid, HEADING_BINS)
    sensing = provider(transition, _site(grid))
    assert not any(state.connector_visible for state in sensing.values())
    assert all(state.covariance_trace > GATE for state in sensing.values())


def test_observability_is_location_dependent():
    """The same station and screen accept one site and reject another."""
    catalog, transition, grid = _setup()
    wx, wy = grid.cell_center(20, 17)
    station = FiducialStation("east", (wx, wy + 3.0, 1.6))
    screen = Box3("screen", (wx, wy + 1.5, 1.0), (1.0, 0.1, 2.0))
    provider = station_sensing_provider(
        catalog, (station,), (screen,), grid, HEADING_BINS)
    blocked = provider(transition, _site(grid, x=20))
    clear = provider(transition, _site(grid, x=40))
    assert not any(state.connector_visible for state in blocked.values())
    assert all(state.connector_visible for state in clear.values())


def test_a_low_obstacle_does_not_occlude_a_raised_station():
    """The raised transition shelf is a driving and relocation obstacle, not a
    sight-line one; only a body in the station's line degrades observability."""
    catalog, transition, grid = _setup()
    wx, wy = grid.cell_center(20, 17)
    station = FiducialStation("east", (wx, wy + 3.0, 1.6))
    shelf = Box3("raised_transition_shelf", (wx, wy + 0.415, 0.14), (0.6, 0.1, 0.08))
    provider = station_sensing_provider(
        catalog, (station,), (shelf,), grid, HEADING_BINS)
    sensing = provider(transition, _site(grid))
    assert all(state.connector_visible for state in sensing.values())


def test_prediction_is_stable_and_cached_per_site():
    catalog, transition, grid = _setup()
    wx, wy = grid.cell_center(20, 17)
    station = FiducialStation("east", (wx, wy + 3.0, 1.6))
    provider = station_sensing_provider(
        catalog, (station,), (), grid, HEADING_BINS)
    first = provider(transition, _site(grid))
    assert provider(transition, _site(grid)) is first


def test_declared_stations_survive_the_manifest_round_trip(tmp_path):
    """The planner node reads stations from the same manifest that carries the
    3D obstacles, so a scenario's stations must serialize and reload intact."""
    import json
    from dataclasses import replace

    from modular_robot_benchmarks.confirmatory_scenarios import (
        make_confirmatory_scenario,
    )
    from morphology_planner import load_fiducial_stations

    scenario = make_confirmatory_scenario("docking_observability", 1, 1)
    station = FiducialStation("staging", (0.3, 1.8, 1.8), max_range=6.0)
    scenario = replace(scenario, fiducial_stations=(station,))
    path = tmp_path / "world.manifest.json"
    path.write_text(json.dumps(scenario.manifest()), encoding="utf-8")
    assert load_fiducial_stations(str(path)) == (station,)


def test_a_scenario_without_stations_round_trips_to_none(tmp_path):
    import json

    from modular_robot_benchmarks.confirmatory_scenarios import (
        make_confirmatory_scenario,
    )
    from morphology_planner import load_fiducial_stations

    scenario = make_confirmatory_scenario("docking_observability", 1, 1)
    path = tmp_path / "world.manifest.json"
    path.write_text(json.dumps(scenario.manifest()), encoding="utf-8")
    assert load_fiducial_stations(str(path)) == ()


def test_covariance_tracks_range_from_the_station():
    catalog, transition, grid = _setup()
    wx, wy = grid.cell_center(20, 17)
    near = station_sensing_provider(
        catalog, (FiducialStation("near", (wx, wy + 1.0, 1.6)),), (),
        grid, HEADING_BINS)(transition, _site(grid))
    far = station_sensing_provider(
        catalog, (FiducialStation("far", (wx, wy + 6.0, 1.6)),), (),
        grid, HEADING_BINS)(transition, _site(grid))
    pod = "pod_0"
    assert near[pod].covariance_trace < far[pod].covariance_trace
    assert far[pod].covariance_trace < GATE
    assert near[pod].covariance_trace > precise_trace(0.0)
