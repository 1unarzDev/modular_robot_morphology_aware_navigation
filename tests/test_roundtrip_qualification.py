from dataclasses import replace

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
