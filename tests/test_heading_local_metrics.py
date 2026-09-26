"""Bind the heading-local estimators to what they were written to measure.

`studies/engineering/heading_traction_curve.json` found that a stage total
cannot measure per-heading authority: a 4 s rotation stage turns the body up to
1 rad, so the total integrates authority over the arc traversed. These tests
build traces whose per-heading behaviour is known by construction and check the
estimators recover it where the stage total does not.
"""
from math import radians

import pytest

from modular_robot_benchmarks.motion_metrics import (
    bin_by_heading, heading_local_rates, time_to_reach)


def _rotation_trace(start_deg, rate_at, command=0.35, duration=4.0, dt=0.05):
    """Integrate a heading-dependent yaw rate into a sample trace."""
    samples = []
    yaw = radians(start_deg)
    time = 0.0
    while time <= duration:
        samples.append({
            "stage": "positive_yaw", "time_s": time, "x": 0.0, "y": 0.0,
            "yaw": yaw, "linear_command": 0.0, "angular_command": command,
            "pod_commands": {"pod_0": 0.0},
        })
        yaw += rate_at(yaw) * dt
        time += dt
    return samples


def test_stage_total_hides_heading_dependence_that_the_estimator_recovers():
    # Authority is full below 45 degrees and absent above it. One stage starts
    # inside that edge and sweeps up to it; one starts beyond it and cannot move.
    def rate_at(yaw):
        return 0.35 if abs(yaw) < radians(45) else 0.0

    below = _rotation_trace(0, rate_at)
    above = _rotation_trace(70, rate_at)
    # The stage totals disagree, which is the confound: one stage spends part of
    # its window in each regime and the other spends all of it in one.
    assert below[-1]["yaw"] > radians(40)
    assert above[-1]["yaw"] == above[0]["yaw"]
    # The estimator attributes each window to the heading it was measured at,
    # so the edge is recovered rather than averaged away.
    table = bin_by_heading(
        heading_local_rates(below + above, ("positive_yaw",), "yaw"),
        width_deg=15.0, fold_deg=None)
    assert table["0"]["median_fraction_of_command"] == pytest.approx(1.0)
    assert table["30"]["median_fraction_of_command"] == pytest.approx(1.0)
    assert table["60"]["median_fraction_of_command"] == pytest.approx(0.0)


def test_folding_is_optional_so_the_period_is_measured_not_assumed():
    points = [(10.0, 0.7), (190.0, 0.7), (100.0, 0.05)]
    folded = bin_by_heading(points, width_deg=20.0, fold_deg=180.0)
    assert folded["0"]["n"] == 2
    unfolded = bin_by_heading(points, width_deg=20.0, fold_deg=None)
    assert unfolded["0"]["n"] == 1 and unfolded["180"]["n"] == 1


def test_time_to_reach_separates_the_ramp_from_the_settled_speed():
    # Same settled speed, different ramps: the totals differ but the speed the
    # stage eventually holds does not, which is the measured signature of a
    # traction limit on acceleration.
    def trace(ramp_s, command=0.2, duration=4.0, dt=0.05):
        samples, x, time = [], 0.0, 0.0
        while time <= duration:
            samples.append({
                "stage": "straight", "time_s": time, "x": x, "y": 0.0,
                "yaw": 0.0, "linear_command": command, "angular_command": 0.0,
                "pod_commands": {"pod_0": command},
            })
            speed = command * min(1.0, time / ramp_s) if ramp_s else command
            x += speed * dt
            time += dt
        return samples

    quick = time_to_reach(trace(0.1), "straight")
    slow = time_to_reach(trace(1.5), "straight")
    assert quick["time_to_target_s"] < slow["time_to_target_s"]
    assert quick["total_translation_m"] > slow["total_translation_m"]
    assert abs(quick["median_speed_after_target"]
               - slow["median_speed_after_target"]) < 0.05
    # And the deficit is a startup cost, so it does not scale with the stage.
    assert 0.9 < (quick["total_translation_m"] - slow["total_translation_m"]) \
        / (0.2 * 1.5 / 2) < 1.1
