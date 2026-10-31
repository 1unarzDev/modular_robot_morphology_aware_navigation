"""Bind the plan-execution measures to traces whose answer is known."""
import math

from modular_robot_benchmarks.execution_metrics import (
    in_place_turns, route_offsets, summarize_record)


def _record(samples, route, narrow_from=0.0):
    return {
        "spec": {"trial_id": "t"}, "terminal_status": "completed",
        "planned_route": [{"x": x, "y": y} for x, y in route],
        "topology_history": [{"time_s": narrow_from, "morphology": "narrow_tandem"}],
        "controller_diagnostics": {"pod_drive_diagnostic_samples": [
            {"pod": pod, "time_s": t, "core_world_x": x, "core_world_y": y,
             "core_world_yaw": yaw, "body_linear_command_mps": v,
             "body_angular_command_radps": w}
            for pod in ("pod_0", "pod_1")
            for t, x, y, yaw, v, w in samples]}}


def test_route_offset_is_distance_to_the_nearest_segment_while_driving():
    route = [(x / 10, 0.0) for x in range(11)]
    samples = [(0.5 * i, 0.1 * i, 0.04, 0.0, 0.18, 0.0) for i in range(10)]
    samples.append((5.0, 0.5, 0.30, 0.0, 0.0, 0.0))       # stopped: not counted
    offsets = route_offsets(_record(samples, route))
    assert len(offsets) == 10
    assert all(abs(offset - 0.04) < 1e-9 for offset in offsets)


def test_a_jump_between_concatenated_plans_is_not_a_route_segment():
    route = [(0.0, 0.0), (0.1, 0.0), (0.0, 1.0), (0.1, 1.0)]
    samples = [(0.0, 0.05, 0.5, 0.0, 0.18, 0.0)]
    assert abs(route_offsets(_record(samples, route))[0] - 0.5) < 1e-9


def test_compact_driving_before_the_transition_is_excluded():
    route = [(0.0, 0.0), (0.1, 0.0)]
    samples = [(1.0, 0.05, 0.2, 0.0, 0.18, 0.0)]
    assert route_offsets(_record(samples, route, narrow_from=5.0)) == []


def test_in_place_turns_split_on_direction_and_wrap():
    samples, yaw, t = [], 3.0, 0.0
    for _ in range(6):                      # +0.6 rad through +pi
        samples.append((t, 0, 0, yaw, 0.0, 0.4)); yaw += 0.1; t += 0.5
    for _ in range(3):                      # -0.3 rad
        samples.append((t, 0, 0, math.atan2(math.sin(yaw), math.cos(yaw)), 0.0, -0.4))
        yaw -= 0.1; t += 0.5
    samples.append((t, 0, 0, math.atan2(math.sin(yaw), math.cos(yaw)), 0.18, 0.0))
    turns = in_place_turns(_record(samples, [(0, 0), (0.1, 0)]))
    assert [round(turn, 6) for turn in turns] == [0.6, -0.3]
    summary = summarize_record(_record(samples, [(0, 0), (0.1, 0)]))
    assert abs(summary["largest_in_place_turn_rad"] - 0.6) < 1e-9
