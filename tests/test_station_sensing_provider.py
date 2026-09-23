"""The planner's prediction of station observability at a candidate site.

ADR 0006: precision docking depends on an external workspace fiducial station,
so a site is well observed only where the station can see where the pods will
be docked. The planner predicts that from priors alone.
"""

from pathlib import Path

import pytest

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


def test_a_shadow_rejects_a_region_dilated_by_the_pods_reach():
    """The granularity floor on any station manipulation.

    A site is rejected when ANY moved pod is shadowed, so the rejected set is
    the screen's shadow dilated by the pods' reach, never equal to it. Realizing
    a declared region therefore needs the shadow eroded by that reach, and a
    region narrower than twice the reach cannot be realized at all.

    compact_to_narrow docks pods at x offsets of +/-0.71 m, so the narrowest
    rejected band this platform can produce is about 1.42 m wide.
    """
    catalog, transition, grid = _setup()
    reach = max(
        abs(catalog.morphologies[transition.target].pod_poses[pod][0])
        for pod in transition.moved_pods)
    assert reach == pytest.approx(0.71, abs=1e-6)

    wx, wy = grid.cell_center(30, 17)
    station = FiducialStation("south", (wx, wy - 3.0, 1.8))
    # A narrow screen, far narrower than the pods' reach.
    screen = Box3("screen", (wx, wy - 1.5, 1.3), (0.10, 0.1, 1.4))
    provider = station_sensing_provider(
        catalog, (station,), (screen,), grid, HEADING_BINS)
    rejected = [
        grid.cell_center(cell_x, 17)[0]
        for cell_x in range(10, 50)
        if not all(state.connector_visible
                   for state in provider(
                       transition, _site(grid, x=cell_x, y=17)).values())
    ]
    assert rejected
    width = max(rejected) - min(rejected)
    assert width >= 2 * reach - grid.resolution, (
        f"a {screen.size[0]:.2f} m screen rejected only {width:.2f} m; the "
        f"floor is {2 * reach:.2f} m")


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
    """The workshop diagnostic worlds declare no station, so they keep the
    pre-ADR-0006 behaviour and must not become station-gated."""
    import json

    from modular_robot_benchmarks.confirmatory_scenarios import (
        make_confirmatory_scenario,
    )
    from morphology_planner import load_fiducial_stations

    scenario = make_confirmatory_scenario("workshop_neutral", 0, 1)
    assert scenario.fiducial_stations == ()
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
