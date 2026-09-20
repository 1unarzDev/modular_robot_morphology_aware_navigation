from dataclasses import replace
from pathlib import Path

import pytest

from modular_robot_benchmarks.design import (
    FAULT_MATRIX, FAULT_MATRIX_DESIGN_KIND, StudyDesign, fault_case_for_run,
    generate_fault_matrix_campaign, generate_fault_matrix_design,
)
from modular_robot_benchmarks.engineering_qualification import (
    evaluate_fault_trial, summarize_fault_matrix,
)
from modular_robot_benchmarks.mission_batch import failure_injection_for
from modular_robot_benchmarks.records import TrialManifest, TrialRecord, TrialStore
from modular_robot_benchmarks.roundtrip_qualification import generate_roundtrip_design
from reconfiguration_executor.control import injected_failure


ROOT = Path(__file__).parents[1]


def test_fault_matrix_design_freezes_with_one_method_and_independent_seeds(tmp_path):
    design = generate_fault_matrix_design("combined_constraints", 5, "geometry_coupled")
    assert design.design_kind == FAULT_MATRIX_DESIGN_KIND
    assert design.expected_trials == len(FAULT_MATRIX)
    assert design.methods == ("geometry_coupled",)
    assert len({spec.world_seed for spec in design.trials}) == 1
    assert len({spec.friction_seed for spec in design.trials}) == len(FAULT_MATRIX)
    path = design.write_frozen(tmp_path / "faults.json")
    assert StudyDesign.read_frozen(path) == design
    with pytest.raises(ValueError, match="family"):
        generate_fault_matrix_design("unknown_family", 5, "geometry_coupled")


def test_checked_in_confirmatory_design_is_unchanged():
    design = StudyDesign.read_frozen(ROOT / "studies/confirmatory/design.json")
    assert design.design_hash == (
        "e9115d543af0969db7825398f7e2691361bdb536530d93e49db4528f21c51cf5")


def test_fault_matrix_declares_every_roadmap_fault_as_a_valid_injection():
    assert {case.name for case in FAULT_MATRIX} == {
        "detach", "relocation", "latch", "stale_observation", "cancellation",
        "commit", "partial_topology"}
    for case in FAULT_MATRIX:
        stage, _, pod = case.injection.partition(":")
        assert injected_failure(case.injection, stage, pod)
    design = generate_fault_matrix_design("combined_constraints", 5, "geometry_coupled")
    assert [failure_injection_for(design, spec) for spec in design.trials] == [
        case.injection for case in FAULT_MATRIX]
    roundtrip = generate_roundtrip_design()
    assert failure_injection_for(roundtrip, roundtrip.trials[0]) == ""
    with pytest.raises(ValueError, match="design kind"):
        summarize_fault_matrix(roundtrip, [])


def _fault_record(design, spec, probe):
    case = fault_case_for_run(spec.replicate)
    return TrialRecord(
        schema_version=1, spec=spec,
        manifest=TrialManifest("abc", "cfg", design.design_hash, {},
                               {"failure_injection": case.injection}),
        terminal_status="unsafe_topology", completed=False,
        simulated_duration_s=40.0, wall_duration_s=80.0,
        final_execution_state="RECOVERY_REQUIRED", unrecovered_fault=True,
        infrastructure_attempts=[{"attempt": 1, "terminal_status": "unsafe_topology"}],
        recovery_probe=probe, notes=["reconfiguration failed; observed topology requires recovery"])


def test_fault_contract_requires_inhibited_drive_and_topology_matched_reconciliation(tmp_path):
    design = generate_fault_matrix_design("combined_constraints", 5, "geometry_coupled")
    records = []
    for spec in design.trials:
        expected = FAULT_MATRIX[spec.replicate].expected_reconciled_morphology
        records.append(_fault_record(design, spec, {
            "performed": True, "state_before": "RECOVERY_REQUIRED",
            "max_pod_command": 0.0, "core_translation_m": 0.0005,
            "core_yaw_change_rad": 0.001,
            "reconcile": ({"success": True, "execution_state": "READY",
                           "morphology": expected} if expected else
                          {"success": False, "execution_state": "RECOVERY_REQUIRED",
                           "morphology": "compact_diff"}),
        }))
    assert summarize_fault_matrix(design, records)["gate_passed"]
    missing = summarize_fault_matrix(design, records[:-1])
    assert not missing["gate_passed"] and missing["cases"][-1]["failed_checks"] == ["pending"]

    partial = next(spec for spec in design.trials
                   if FAULT_MATRIX[spec.replicate].name == "partial_topology")
    moved = _fault_record(design, partial, {
        **records[partial.replicate].recovery_probe, "max_pod_command": 0.1,
        "core_translation_m": 0.05})
    unsafe_reconcile = _fault_record(design, partial, {
        **records[partial.replicate].recovery_probe,
        "reconcile": {"success": True, "execution_state": "READY",
                      "morphology": "compact_diff"}})
    case = FAULT_MATRIX[partial.replicate]
    assert evaluate_fault_trial(moved, case)["failed_checks"] == ["drive_inhibited"]
    assert evaluate_fault_trial(unsafe_reconcile, case)["failed_checks"] == [
        "reconciliation_matches_observed_topology"]
    store = TrialStore(tmp_path / "raw")
    timing = {"readiness_wall_s": 12.5, "mission_wall_s": 40.0,
              "real_time_factor": 1.0, "setup_wall_s": 0.4, "teardown_wall_s": 3.2}
    store.write_terminal(replace(moved, phase_timing=timing))
    loaded = store.load_all()[0]
    assert loaded.recovery_probe["max_pod_command"] == 0.1
    assert loaded.phase_timing == timing


def test_demo_launch_forwards_declared_failure_injection_to_executor():
    launch = (ROOT / "src/modular_robot_bringup/launch/demo.launch.py").read_text()
    assert 'DeclareLaunchArgument(\n            "failure_injection"' in launch
    assert '"failure_injection": ParameterValue(failure_injection, value_type=str)' in launch


def test_cross_layout_campaign_preserves_the_single_layout_design_hash():
    """Generalizing the matrix must not invalidate the matrix that passed.

    `docs/current_status.md` records the passing seven-case execution against
    a specific design hash; a one-layout, one-realization campaign is that
    design.
    """
    single = generate_fault_matrix_design("reconfiguration_workspace", 0,
                                          "sensing_feasibility_coupled", 1)
    campaign = generate_fault_matrix_campaign(
        "reconfiguration_workspace", (0,), "sensing_feasibility_coupled", 1, 1)
    assert single == campaign
    assert single.design_hash == (
        "f1e172305fe3fb29f98a7fb54468fea16885090093a04111e1aaf0e73cfcf29e")


def test_campaign_repeats_every_declared_case_in_every_layout(tmp_path):
    campaign = generate_fault_matrix_campaign(
        "reconfiguration_workspace", (0, 3, 7), "sensing_feasibility_coupled",
        realizations=2)
    assert campaign.expected_trials == len(FAULT_MATRIX) * 2 * 3
    assert len({spec.layout_id for spec in campaign.trials}) == 3
    # One world per layout, but never a repeated fault or plant realization.
    assert len({spec.world_seed for spec in campaign.trials}) == 3
    for field in ("fault_seed", "friction_seed", "sensing_seed"):
        seeds = [getattr(spec, field) for spec in campaign.trials]
        assert len(set(seeds)) == len(seeds), field
    assert StudyDesign.read_frozen(
        campaign.write_frozen(tmp_path / "campaign.json")) == campaign
    with pytest.raises(ValueError, match="distinct"):
        generate_fault_matrix_campaign(
            "reconfiguration_workspace", (0, 0), "sensing_feasibility_coupled")
    with pytest.raises(ValueError, match="at least one layout"):
        generate_fault_matrix_campaign(
            "reconfiguration_workspace", (), "sensing_feasibility_coupled")


def _passing_campaign_records(design):
    records = []
    for spec in design.trials:
        expected = fault_case_for_run(spec.replicate).expected_reconciled_morphology
        records.append(_fault_record(design, spec, {
            "performed": True, "state_before": "RECOVERY_REQUIRED",
            "max_pod_command": 0.0, "core_translation_m": 0.0005,
            "core_yaw_change_rad": 0.001,
            "reconcile": ({"success": True, "execution_state": "READY",
                           "morphology": expected} if expected else
                          {"success": False, "execution_state": "RECOVERY_REQUIRED",
                           "morphology": "compact_diff"}),
        }))
    return records


def test_campaign_summary_reports_each_layout_and_refuses_a_narrowed_matrix():
    design = generate_fault_matrix_campaign(
        "reconfiguration_workspace", (0, 3), "sensing_feasibility_coupled",
        realizations=2)
    summary = summarize_fault_matrix(design, _passing_campaign_records(design))
    assert summary["gate_passed"]
    assert summary["layouts"] == ["reconfiguration_workspace-00",
                                  "reconfiguration_workspace-03"]
    assert summary["realizations_per_case"] == 2
    assert summary["distinct_fault_seeds"]
    assert all(value["passed"] for value in summary["by_layout"].values())
    assert {case["realization"] for case in summary["cases"]} == {0, 1}

    # One failing layout fails the campaign, and is attributed to that layout.
    records = _passing_campaign_records(design)
    broken = next(index for index, spec in enumerate(design.trials)
                  if spec.layout_id.endswith("-03"))
    records[broken] = replace(
        records[broken],
        recovery_probe={**records[broken].recovery_probe, "max_pod_command": 0.2})
    failed = summarize_fault_matrix(design, records)
    assert not failed["gate_passed"]
    assert failed["by_layout"]["reconfiguration_workspace-00"]["passed"]
    assert not failed["by_layout"]["reconfiguration_workspace-03"]["passed"]


def test_every_campaign_trial_maps_to_its_declared_injection():
    """Runtime injection must follow the same run->case map as the auditor.

    A multi-realization campaign has run indices past the end of FAULT_MATRIX;
    indexing it directly would fail at launch, in the container, mid-campaign.
    """
    design = generate_fault_matrix_campaign(
        "reconfiguration_workspace", (0, 3), "sensing_feasibility_coupled",
        realizations=3)
    injections = [failure_injection_for(design, spec) for spec in design.trials]
    assert all(injections)
    assert set(injections) == {case.injection for case in FAULT_MATRIX}
    for spec in design.trials:
        assert (failure_injection_for(design, spec)
                == fault_case_for_run(spec.replicate).injection)
