from dataclasses import replace

import pytest

from modular_robot_benchmarks.records import TrialManifest, TrialRecord
from modular_robot_benchmarks.roundtrip_qualification import (
    audit_roundtrip_records, generate_roundtrip_design,
)


def _passing_records(design):
    records = []
    for index, spec in enumerate(design.trials):
        qualification = {
            "passed": True, "pod_rigidity": {"passed": True}, "stages": {}}
        records.append(TrialRecord(
            schema_version=1, spec=spec,
            manifest=TrialManifest("abc", "config", design.design_hash, {}, {
                "physical_disturbance": {"ground_friction": 0.7 + index / 100.0}}),
            terminal_status="completed", completed=True,
            simulated_duration_s=100.0, wall_duration_s=10.0,
            reconfiguration_attempts=2, motion_qualifications=[qualification, qualification],
            topology_history=[
                {"morphology": "compact_diff"}, {"morphology": "narrow_tandem"},
                {"morphology": "compact_diff"}],
            final_morphology="compact_diff", final_execution_state="READY"))
    return records


def test_roundtrip_design_has_twenty_unique_paired_disturbance_runs():
    design = generate_roundtrip_design()
    assert len(design.trials) == 20
    assert {spec.method for spec in design.trials} == {"feasibility_coupled"}
    assert len({spec.friction_seed for spec in design.trials}) == 20


def test_roundtrip_audit_requires_complete_motion_rigidity_and_topology_contract():
    design = generate_roundtrip_design()
    records = _passing_records(design)
    assert audit_roundtrip_records(records, design)["passed"]
    broken = replace(records[3], motion_qualifications=[{
        "passed": True, "pod_rigidity": {"passed": False}, "stages": {}}])
    result = audit_roundtrip_records([*records[:3], broken, *records[4:]], design)
    assert not result["passed"]
    assert any("pod_rigidity" in failure for failure in result["failures"])


def _stages_at(scale=1.0):
    """A comfortably passing signed-motion set, scaled toward the bounds."""
    return {
        "positive_yaw": {"yaw_rad": 0.45 * scale, "translation_m": 0.006,
                         "forward_m": -0.004, "lateral_m": -0.002},
        "negative_yaw": {"yaw_rad": -0.51 * scale, "translation_m": 0.003,
                         "forward_m": 0.002, "lateral_m": 0.002},
        "forward": {"forward_m": 0.113 * scale, "lateral_m": -0.001,
                    "yaw_rad": -0.012, "translation_m": 0.113 * scale},
        "reverse": {"forward_m": -0.113 * scale, "lateral_m": 0.0004,
                    "yaw_rad": 0.0005, "translation_m": 0.113 * scale},
    }


def test_declared_motion_bounds_match_the_gate_that_enforces_them():
    """A margin reported against a bound the gate does not enforce is fiction.

    Each declared bound is driven just past its limit and the real
    `qualification_pass` must reject, then pulled just inside and it must
    accept. That binds the two tables without duplicating the predicate.

    The two yaw bounds interact, so each probe has to isolate the one it is
    testing. Absolute yaw translation (0.12 m) only binds above 0.48 rad of
    yaw; below that the 0.25 coupling ratio is the tighter constraint. The
    probes set the stage yaw accordingly.
    """
    from modular_robot_bringup.qualification import qualification_pass
    from modular_robot_benchmarks.roundtrip_qualification import MOTION_BOUNDS

    assert qualification_pass(_stages_at())
    epsilon = 1e-4
    for label, stage, field, direction, bound in MOTION_BOUNDS:
        failing, passing = _stages_at(), _stages_at()
        sign = -1.0 if stage == "negative_yaw" else 1.0
        if field == "translation_m":
            for stages in (failing, passing):
                stages[stage]["yaw_rad"] = sign * 0.60
            failing[stage][field] = bound + epsilon
            passing[stage][field] = bound - epsilon
        elif field == "translation_per_yaw":
            yaw = 0.40
            for stages in (failing, passing):
                stages[stage]["yaw_rad"] = sign * yaw
            failing[stage]["translation_m"] = (bound + epsilon) * yaw
            passing[stage]["translation_m"] = (bound - epsilon) * yaw
        elif direction == "min":
            failing[stage][field], passing[stage][field] = bound - epsilon, bound + epsilon
        else:
            failing[stage][field], passing[stage][field] = bound + epsilon, bound - epsilon
        assert not qualification_pass(failing), f"{label} is not enforced at {bound}"
        assert qualification_pass(passing), f"{label} rejects inside its own bound {bound}"


def test_absolute_yaw_translation_bound_is_slack_below_half_a_radian():
    """Documents why the probes above vary yaw, and that the bound is real.

    `translation_m <= 0.12` cannot bind until yaw exceeds 0.12/0.25 = 0.48 rad.
    The engineering runs yaw about 0.45 rad, so in practice the coupling ratio
    is the constraint that rejects first and the absolute bound is slack.
    """
    from modular_robot_bringup.qualification import qualification_pass

    at_observed_yaw = _stages_at()
    at_observed_yaw["positive_yaw"] = {"yaw_rad": 0.45, "translation_m": 0.119,
                                       "forward_m": 0.0, "lateral_m": 0.0}
    assert not qualification_pass(at_observed_yaw)
    above_the_crossover = _stages_at()
    above_the_crossover["positive_yaw"] = {"yaw_rad": 0.60, "translation_m": 0.119,
                                           "forward_m": 0.0, "lateral_m": 0.0}
    assert qualification_pass(above_the_crossover)



def test_audit_reports_worst_case_motion_margins_and_the_load_it_met():
    design = generate_roundtrip_design()
    records = _passing_records(design)
    tight = _stages_at()
    tight["forward"]["forward_m"] = 0.09
    for index, record in enumerate(records):
        stages = tight if index == 3 else _stages_at()
        record.motion_qualifications[:] = [
            {"passed": True, "pod_rigidity": {"passed": True}, "stages": stages}]
        record.phase_timing.update({"real_time_factor": 0.5 + index / 100.0})
    result = audit_roundtrip_records(records, design)
    forward = result["motion_margins"]["forward_travel_m"]
    assert forward["bound"] == 0.08
    assert forward["observed_min"] == 0.09
    # Worst margin is the tightest run, not the typical one.
    assert forward["worst_margin"] == pytest.approx(0.01)
    assert forward["median_margin"] == pytest.approx(0.033)
    assert result["real_time_factors"]["minimum"] == pytest.approx(0.5)
    assert result["real_time_factors"]["maximum"] == pytest.approx(0.69)
