import json

from morphology_planner.observability import (
    STATION_UNOBSERVED_TRACE, FiducialStation, load_fiducial_stations,
    observed_by_any, precise_trace, segment_intersects_box, station_observes,
)
from morphology_planner.transition_validation import Box3

# The acceptance gate shared by the planner predicate and `docking_acceptance`.
GATE = 0.015

STATION = FiducialStation("station", (0.0, 0.0, 1.0), max_range=8.0)
POD = (2.0, 0.0, 0.1)


def box(name, center, size=(0.2, 0.2, 1.0)):
    return Box3(name, center, size)


def test_clear_sight_line_is_observed():
    assert station_observes(STATION, POD, ()) is True


def test_obstacle_on_the_sight_line_blocks_it():
    assert segment_intersects_box(STATION.position, POD, box("post", (1.0, 0.0, 0.5)))
    assert station_observes(STATION, POD, (box("post", (1.0, 0.0, 0.5)),)) is False


def test_obstacle_beside_the_sight_line_does_not_block_it():
    assert station_observes(STATION, POD, (box("post", (1.0, 0.5, 0.5)),)) is True


def test_obstacle_beyond_the_pod_does_not_block_it():
    """The sight line is a segment, not a ray: nothing past the pod occludes."""
    assert station_observes(STATION, POD, (box("post", (3.0, 0.0, 0.5)),)) is True


def test_obstacle_behind_the_station_does_not_block_it():
    assert station_observes(STATION, POD, (box("post", (-1.0, 0.0, 0.5)),)) is True


def test_obstacle_below_the_sight_line_does_not_block_it():
    """Height matters: a low shelf under a raised station's view stays clear."""
    assert station_observes(
        STATION, POD, (box("shelf", (1.0, 0.0, 0.1), (0.4, 0.4, 0.2)),)) is True


def test_grazing_a_face_is_not_an_occlusion():
    assert station_observes(STATION, POD, (box("post", (1.0, 0.1, 0.5)),)) is True


def test_a_pod_out_of_range_is_not_observed():
    near_sighted = FiducialStation("station", (0.0, 0.0, 1.0), max_range=1.0)
    assert near_sighted.range_to(POD) > 1.0
    assert station_observes(near_sighted, POD, ()) is False


def test_observed_by_any_returns_the_nearest_unblocked_station():
    far = FiducialStation("far", (6.0, 0.0, 1.0))
    near = FiducialStation("near", (0.0, 0.0, 1.0))
    assert observed_by_any((far, near), POD, ()) is near


def test_observed_by_any_skips_a_blocked_station():
    far = FiducialStation("far", (6.0, 0.0, 1.0))
    near = FiducialStation("near", (0.0, 0.0, 1.0))
    blocker = box("post", (1.0, 0.0, 0.5))
    assert observed_by_any((far, near), POD, (blocker,)) is far


def test_observed_by_any_returns_none_when_every_station_is_blocked():
    near = FiducialStation("near", (0.0, 0.0, 1.0))
    assert observed_by_any((near,), POD, (box("post", (1.0, 0.0, 0.5)),)) is None


def test_station_refined_covariance_passes_the_gate_at_working_range():
    # compact_to_narrow places its moved pods at most 0.74 m from the core.
    assert precise_trace(0.74) < GATE
    assert precise_trace(8.0) < GATE


def test_unobserved_covariance_rejects_at_the_gate():
    assert STATION_UNOBSERVED_TRACE > GATE


def test_no_manifest_means_no_station_and_no_gating():
    assert load_fiducial_stations("") == ()


def test_a_manifest_without_stations_declares_none(tmp_path):
    path = tmp_path / "world.manifest.json"
    path.write_text(json.dumps({"transition_obstacles": []}), encoding="utf-8")
    assert load_fiducial_stations(str(path)) == ()


def test_declared_stations_are_loaded_with_their_range(tmp_path):
    path = tmp_path / "world.manifest.json"
    path.write_text(json.dumps({"fiducial_stations": [
        {"name": "east", "position": [1.0, 2.0, 1.5], "max_range": 4.0},
        {"name": "west", "position": [5.0, 2.0, 1.5]},
    ]}), encoding="utf-8")
    stations = load_fiducial_stations(str(path))
    assert [station.name for station in stations] == ["east", "west"]
    assert stations[0].position == (1.0, 2.0, 1.5)
    assert stations[0].max_range == 4.0
    assert stations[1].max_range == 8.0
