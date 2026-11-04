import pytest

from modular_robot_benchmarks.prevalence import (
    KEEP_CLEAR_M, RESOLUTION, make_prevalence_layout, summarize, wilson_interval,
    world_seeds,
)


def test_prevalence_layouts_are_deterministic_and_keep_spawn_and_door_clear():
    for seed in world_seeds(7, 20):
        first, second = make_prevalence_layout(seed), make_prevalence_layout(seed)
        assert first["environment"] == second["environment"]
        assert first["grid"].data == second["grid"].data
        sx, sy = [(value + 0.5) * RESOLUTION for value in first["start"]]
        for box in first["environment"]:
            (cx, cy, cz), (lx, ly, lz) = box.center, box.size
            assert abs(cx - sx) > KEEP_CLEAR_M + lx / 2 or abs(cy - sy) > KEEP_CLEAR_M + ly / 2
            if box.name.startswith("raised"):
                assert 0.12 < cz - lz / 2 < 0.30
        (station,) = first["stations"]
        assert station.position[2] == pytest.approx(1.8)


def test_prevalence_summary_counts_rejected_control_sites():
    def entry(feasible, stages=()):
        return {"planned": True, "site": [1, 1, 0], "full_test_feasible": feasible,
                "failed_stages": list(stages)}
    results = [
        {"furniture": {"tall": 0, "raised": 1}, "methods": {
            "geometry_coupled": entry(False, ["module_trajectory"]),
            "feasibility_coupled": entry(True),
            "sensing_feasibility_coupled": {"planned": True, "site": [2, 2, 0]}}},
        {"furniture": {"tall": 1, "raised": 0}, "methods": {
            "geometry_coupled": entry(True), "feasibility_coupled": entry(False, ["observability"]),
            "sensing_feasibility_coupled": {"planned": False, "error": "NoPathError"}}},
    ]
    summary = summarize(results)
    assert summary["geometry_coupled"]["site_rejected_by_full_test"] == 1
    assert summary["geometry_coupled"]["rejected_and_treatment_plans"] == 1
    assert summary["feasibility_coupled"]["rejected_by_stage"] == {"observability": 1}
    assert summary["feasibility_coupled"]["rejected_and_treatment_plans"] == 0
    assert summary["sensing_feasibility_coupled"] == {"planned": 1, "unplanned": 1}
    low, high = wilson_interval(1, 2)
    assert 0.09 < low < 0.1 and 0.9 < high < 0.91
