"""Gate 1: a record must carry every measured outcome, synchronized and attributable.

The audit was first run over the 62 retained Gate 0 records; every telemetry
check passed and every record lacked exactly the two fields this change adds,
container image and process health.
"""

from dataclasses import replace

from modular_robot_benchmarks.evaluator_metrics import localization_errors
from modular_robot_benchmarks.gate1_audit import (
    audit_record, audit_records, REQUIRED_OUTCOME_CLASSES,
)
from modular_robot_benchmarks.mission_batch import stack_process_deaths


def _diagnostic(time_s):
    return {"time_s": time_s, "pod": "pod_0", "core_world_x": 0.0, "core_world_y": 0.0}


def _record(status="completed", duration=20.0, **changes):
    from modular_robot_benchmarks.design import generate_fault_matrix_design
    from modular_robot_benchmarks.records import TrialManifest, TrialRecord

    design = generate_fault_matrix_design("combined_constraints", 1, "geometry_coupled")
    estimates = [{"time_s": 0.5 * i, "x": 0.0, "y": 0.0} for i in range(int(duration * 2))]
    truth = [_diagnostic(0.25 * i) for i in range(int(duration * 4) + 1)]
    record = TrialRecord(
        schema_version=1, spec=design.trials[0],
        manifest=TrialManifest("a" * 40, "cfg", design.design_hash,
                               {"container_image": "sha256:feed"}, {}),
        terminal_status=status, completed=status == "completed",
        simulated_duration_s=duration, wall_duration_s=duration,
        mechanical_work_j=12.5, covariance_trace=[0.01],
        predicted_transition_probabilities=[0.9], observed_transition_outcomes=[1],
        infrastructure_attempts=[{"attempt": 1, "terminal_status": status,
                                  "stack_process_deaths": []}],
        localization_history=estimates,
        localization_error_m=localization_errors(estimates, truth),
        controller_diagnostics={"pod_drive_diagnostic_samples": truth,
                                "pod_drive_diagnostic_samples_dropped": 0,
                                "evaluator_clearance": {"evaluated_samples": 3}},
    )
    return replace(record, **changes)


def test_a_complete_record_has_no_problems():
    assert audit_record(_record())["problems"] == []


def test_missing_provenance_is_rejected():
    from modular_robot_benchmarks.records import TrialManifest

    record = _record()
    dirty = replace(record, manifest=TrialManifest(
        "abc-dirty", "", record.manifest.design_hash, {}, {}))
    problems = audit_record(dirty)["problems"]
    assert any("not a clean commit" in p for p in problems)
    assert "configuration hash missing" in problems
    assert "container image not recorded" in problems


def test_each_missing_mission_measurement_is_named():
    final = {"attempt": 1, "terminal_status": "completed"}
    cases = {
        "no estimate covariance trace": {"covariance_trace": []},
        "mechanical work not measured": {"mechanical_work_j": 0.0},
        "1 transition predictions against 0 observed outcomes": {
            "observed_transition_outcomes": []},
        "process health not recorded": {"infrastructure_attempts": [final]},
        "evaluator clearance was not computed": {"controller_diagnostics": {
            key: value for key, value in _record().controller_diagnostics.items()
            if key != "evaluator_clearance"}},
    }
    for expected, change in cases.items():
        assert expected in audit_record(_record(**change))["problems"], expected


def test_a_stack_death_during_the_trial_is_a_problem():
    deaths = [{"process": "amcl-4", "exit_code": -11}]
    record = _record(infrastructure_attempts=[
        {"attempt": 1, "terminal_status": "completed", "stack_process_deaths": deaths}])
    assert "stack processes died during the trial: amcl-4" in audit_record(record)["problems"]


def test_a_record_without_a_mission_is_checked_for_provenance_only():
    row = audit_record(_record(status="infrastructure_failure", duration=0.0,
                               covariance_trace=[], mechanical_work_j=0.0))
    assert row["has_mission"] is False
    assert row["problems"] == []


def test_the_set_passes_only_when_clean_and_every_outcome_class_is_covered():
    statuses = ["completed", "collision", "localization_lost", "unsafe_topology", "timeout"]
    summary = audit_records([_record(status=s) for s in statuses])
    assert summary["gate_passed"] is True
    assert summary["required_classes_missing"] == []
    partial = audit_records([_record(status=s) for s in statuses[:2]])
    assert partial["gate_passed"] is False
    assert partial["required_classes_missing"] == [
        "docking_failure", "localization_loss", "timeout"]
    assert set(REQUIRED_OUTCOME_CLASSES) == set(summary["clean_records_per_required_class"])


def test_stack_deaths_are_read_from_the_launch_log(tmp_path):
    log = tmp_path / "launch.log"
    log.write_text(
        "[INFO] [amcl-4]: process started with pid [11]\n"
        "[ERROR] [amcl-4]: process has died [pid 11, exit code -11, cmd 'amcl']\n"
        "[INFO] [map_server-3]: process has finished cleanly [pid 12]\n",
        encoding="utf-8")
    assert stack_process_deaths(log) == [{"process": "amcl-4", "exit_code": -11}]
    assert stack_process_deaths(tmp_path / "absent.log") == []


def test_the_batch_reads_deaths_before_teardown_kills_the_stack():
    from pathlib import Path

    source = (Path(__file__).parents[1] / "src/modular_robot_benchmarks/"
              "modular_robot_benchmarks/mission_batch.py").read_text()
    assert source.index("premature_deaths = stack_process_deaths(") < source.index(
        "_stop_process(process)")
    assert '"stack_process_deaths": premature_deaths' in source
    assert '"MORPHOLOGY_CONTAINER_IMAGE"' in source
