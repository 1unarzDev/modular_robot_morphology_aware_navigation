"""ADR 0006: the station must deliver the observability the planner predicted.

A site is chosen because the workspace station is predicted to see where the
pods will dock. These cover the two ways that can be wrong: the planner
choosing a site the station cannot see, and the mission recording sensing that
contradicts the prediction.

"""

from pathlib import Path

from modular_robot_benchmarks.confirmatory_scenarios import (
    make_confirmatory_scenario,
)
from modular_robot_benchmarks.evaluator_metrics import (
    STATION_OBSERVABILITY_GATE, station_observability_audit,
)
from morphology_planner import (
    HybridState, load_catalog, station_sensing_provider,
)

CATALOG = Path(__file__).parents[1] / "src/modular_robot_description/config/morphologies.yaml"
PRECISE_VARIANCE = 8.3e-06


class _Record:
    """The two record fields the audit reads."""

    def __init__(self, sites, alignment):
        self.planned_transition_sites = sites
        self.pod_alignment_history = alignment


def _catalog():
    return load_catalog(CATALOG).supported_experiment_subset()


def _transition(catalog):
    return next(t for t in catalog.transitions if t.id == "compact_to_narrow")


def _station_scenario():
    """A confirmatory layout, which under ADR 0006 carries a station and the
    elevated screen that shadows its declared region."""
    scenario = make_confirmatory_scenario("docking_observability", 1, 1)
    assert scenario.fiducial_stations
    return scenario


def _cells_by_visibility(scenario, catalog):
    """One observed and one shadowed cell, found as the planner would."""
    provider = station_sensing_provider(
        catalog, scenario.fiducial_stations, scenario.transition_obstacles,
        scenario.grid, 8)
    transition = _transition(catalog)
    observed = shadowed = None
    for cell_x in range(8, scenario.grid.width - 8):
        for cell_y in range(6, scenario.grid.height - 6):
            sensing = provider(
                transition, HybridState(cell_x, cell_y, 0, "compact_diff"))
            if all(state.connector_visible for state in sensing.values()):
                observed = observed or (cell_x, cell_y)
            else:
                shadowed = shadowed or (cell_x, cell_y)
    assert observed and shadowed, "the fixture must produce both cases"
    return observed, shadowed


def _pods(catalog, visible=True, variance=PRECISE_VARIANCE):
    return {pod: {"connector_visible": visible, "variance_x": variance,
                  "variance_y": variance, "variance_yaw": variance}
            for pod in _transition(catalog).moved_pods}


def _site(scenario, cell, yaw=0.0):
    wx, wy = scenario.grid.cell_center(*cell)
    return {"transition_id": "compact_to_narrow", "x": wx, "y": wy, "yaw": yaw}


def test_a_world_without_a_station_is_exempt():
    """The workshop diagnostic worlds declare no station, so they keep the
    pre-ADR-0006 behaviour and the audit has nothing to check."""
    catalog = _catalog()
    scenario = make_confirmatory_scenario("workshop_neutral", 0, 1)
    assert scenario.fiducial_stations == ()
    record = _Record([_site(scenario, (20, 18))], [])
    assert station_observability_audit(record, scenario, catalog) == []


def test_an_agreeing_mission_raises_nothing():
    catalog = _catalog()
    scenario = _station_scenario()
    observed, _ = _cells_by_visibility(scenario, catalog)
    record = _Record(
        [_site(scenario, observed)],
        [{"execution_state": "READY", "pods": _pods(catalog)}],
    )
    assert station_observability_audit(record, scenario, catalog) == []


def test_a_site_the_station_cannot_see_is_reported():
    """If a mission planned inside the shadow, the prediction that justified
    the site did not hold."""
    catalog = _catalog()
    scenario = _station_scenario()
    _, shadowed = _cells_by_visibility(scenario, catalog)
    record = _Record(
        [_site(scenario, shadowed)],
        [{"execution_state": "READY", "pods": _pods(catalog)}],
    )
    problems = station_observability_audit(record, scenario, catalog)
    assert problems and "cannot see" in problems[0]


def test_measured_invisibility_contradicting_the_prediction_is_reported():
    catalog = _catalog()
    scenario = _station_scenario()
    observed, _ = _cells_by_visibility(scenario, catalog)
    record = _Record(
        [_site(scenario, observed)],
        [{"execution_state": "TRANSITIONING",
          "pods": _pods(catalog, visible=False)}],
    )
    problems = station_observability_audit(record, scenario, catalog)
    assert len(problems) == 6
    assert all("predicted to see" in problem for problem in problems)


def test_measured_covariance_above_the_gate_is_reported():
    catalog = _catalog()
    scenario = _station_scenario()
    observed, _ = _cells_by_visibility(scenario, catalog)
    record = _Record(
        [_site(scenario, observed)],
        [{"execution_state": "READY",
          "pods": _pods(catalog, variance=STATION_OBSERVABILITY_GATE)}],
    )
    problems = station_observability_audit(record, scenario, catalog)
    assert len(problems) == 6
    assert "within the gate" in problems[0]


def test_one_accepted_observation_is_enough():
    """The docking gate needs a single accepted estimate, not every sample, so
    a noisy early sample must not fail the audit."""
    catalog = _catalog()
    scenario = _station_scenario()
    observed, _ = _cells_by_visibility(scenario, catalog)
    record = _Record(
        [_site(scenario, observed)],
        [{"execution_state": "TRANSITIONING",
          "pods": _pods(catalog, visible=False)},
         {"execution_state": "READY", "pods": _pods(catalog)}],
    )
    assert station_observability_audit(record, scenario, catalog) == []


def test_a_mission_that_recorded_no_alignment_snapshots_is_not_penalised():
    """A trial that failed before transitioning has nothing to compare."""
    catalog = _catalog()
    scenario = _station_scenario()
    observed, _ = _cells_by_visibility(scenario, catalog)
    record = _Record([_site(scenario, observed)], [])
    assert station_observability_audit(record, scenario, catalog) == []
