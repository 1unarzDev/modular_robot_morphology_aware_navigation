"""Bind the in-place turn-rate estimator to the windows it was declared over."""
from modular_robot_benchmarks.turn_rate import in_place_turn_ratios, summarize


def _record(rate_per_command, command=0.4, linear=0.0, dt=0.5, steps=11,
            morph_time=0.0):
    samples, yaw = [], 0.0
    for index in range(steps):
        samples.append({
            "pod": "pod_0", "time_s": index * dt, "core_world_yaw": yaw,
            "body_angular_command_radps": command,
            "body_linear_command_mps": linear})
        samples.append({"pod": "pod_1", "time_s": index * dt,
                        "core_world_yaw": 99.0,
                        "body_angular_command_radps": command,
                        "body_linear_command_mps": linear})
        yaw += rate_per_command * command * dt
    return {
        "topology_history": [{"time_s": morph_time,
                              "morphology": "narrow_tandem"}],
        "controller_diagnostics": {"pod_drive_diagnostic_samples": samples}}


def test_recovers_the_constructed_ratio_from_pod_0_only():
    ratios = in_place_turn_ratios(_record(1.8))
    assert len(ratios) == 10
    assert all(name == "narrow_tandem" and command == 0.4
               and abs(ratio - 1.8) < 1e-9 for name, command, ratio in ratios)


def test_ignores_translating_slow_and_mislabelled_windows():
    assert in_place_turn_ratios(_record(1.0, linear=0.05)) == []
    assert in_place_turn_ratios(_record(1.0, command=0.1)) == []
    assert in_place_turn_ratios(_record(1.0, dt=1.0)) == []
    before_transition = in_place_turn_ratios(_record(1.0, morph_time=100.0))
    assert {name for name, _, _ in before_transition} == {"compact_diff"}


def test_yaw_wraps_across_pi():
    record = _record(1.0, command=-0.4)
    for sample in record["controller_diagnostics"][
            "pod_drive_diagnostic_samples"]:
        sample["core_world_yaw"] = (sample["core_world_yaw"] - 3.0)
        if sample["core_world_yaw"] < -3.141592653589793:
            sample["core_world_yaw"] += 2 * 3.141592653589793
    ratios = [ratio for _, _, ratio in in_place_turn_ratios(record)]
    assert all(abs(ratio - 1.0) < 1e-9 for ratio in ratios)


def test_declared_acceptance_rule():
    assert summarize([_record(1.0)])["turn_rate_criteria_pass"]
    assert not summarize([_record(1.8)])["turn_rate_criteria_pass"]
    assert not summarize([_record(0.7)])["turn_rate_criteria_pass"]
    stalls = [_record(1.0)] * 8 + [_record(0.3)] * 2
    summary = summarize(stalls)
    assert abs(summary["pooled"]["median"] - 1.0) < 1e-9
    assert not summary["turn_rate_criteria_pass"]
    assert not summarize([])["turn_rate_criteria_pass"]
