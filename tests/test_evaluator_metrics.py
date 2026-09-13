from pathlib import Path

import pytest

from modular_robot_benchmarks.evaluator_metrics import core_truth_track, localization_errors


ROOT = Path(__file__).resolve().parents[1]
# Packages whose nodes perform localization, planning, docking, or recovery.
AUTONOMY_PACKAGES = (
    "morphology_planner", "morphology_manager", "reconfiguration_executor",
    "modular_robot_bringup", "modular_robot_description",
)
# Gazebo world-pose truth, including the bridged topology pose that lacks the
# /evaluator prefix and the JSON fields carried by evaluator diagnostics.
GROUND_TRUTH_MARKERS = (
    "/evaluator", "module_pose", "topology_joint/pose", "core_world_", "drive_diagnostics",
)


def _diagnostic(time_s, x, y, pod="pod_0"):
    return {"time_s": time_s, "pod": pod, "core_world_x": x, "core_world_y": y}


def test_localization_error_matches_nearest_truth_within_tolerance():
    truth = [_diagnostic(1.0, 0.0, 0.0), _diagnostic(1.1, 0.0, 0.0, "pod_1"),
             _diagnostic(2.0, 1.0, 0.0), _diagnostic(3.0, 2.0, 0.0)]
    estimates = [
        {"time_s": 1.05, "x": 0.03, "y": 0.04, "yaw": 0.0},   # 0.05 m error
        {"time_s": 2.1, "x": 1.0, "y": 0.2, "yaw": 0.0},      # 0.20 m error
        {"time_s": 5.0, "x": 2.0, "y": 0.0, "yaw": 0.0},      # no truth nearby
    ]
    errors = localization_errors(estimates, truth)
    assert errors == pytest.approx([0.05, 0.2])


def test_truth_track_ignores_samples_without_core_pose_and_duplicate_ticks():
    samples = [_diagnostic(1.0, 0.0, 0.0), _diagnostic(1.0, 0.0, 0.0, "pod_3"),
               {"time_s": 1.5, "pod": "pod_2", "world_x": 9.0, "world_y": 9.0}]
    assert core_truth_track(samples) == [(1.0, 0.0, 0.0)]
    assert localization_errors([{"time_s": 1.5, "x": 0.0, "y": 0.0}], samples,
                               tolerance_s=0.1) == []


def _record(duration=20.0, estimates=None, truth=None, dropped=0, errors=None):
    from modular_robot_benchmarks.design import generate_fault_matrix_design
    from modular_robot_benchmarks.records import TrialManifest, TrialRecord

    design = generate_fault_matrix_design("combined_constraints", 1, "geometry_coupled")
    if estimates is None:
        estimates = [{"time_s": 0.5 * i, "x": 0.0, "y": 0.0} for i in range(int(duration * 2))]
    if truth is None:
        truth = [_diagnostic(0.25 * i, 0.0, 0.0) for i in range(int(duration * 4) + 1)]
    return TrialRecord(
        schema_version=1, spec=design.trials[0],
        manifest=TrialManifest("abc", "cfg", design.design_hash, {}, {}),
        terminal_status="completed", completed=True, simulated_duration_s=duration,
        wall_duration_s=duration, final_execution_state="READY",
        infrastructure_attempts=[{"attempt": 1, "terminal_status": "completed"}],
        localization_history=estimates,
        localization_error_m=(localization_errors(estimates, truth) if errors is None else errors),
        controller_diagnostics={"pod_drive_diagnostic_samples": truth,
                                "pod_drive_diagnostic_samples_dropped": dropped})


def test_telemetry_audit_accepts_dense_synchronized_streams():
    from modular_robot_benchmarks.evaluator_metrics import telemetry_audit

    assert telemetry_audit(_record()) == []
    # Pre-navigation infrastructure failures have no mission to measure.
    assert telemetry_audit(_record(duration=0.0, estimates=[], truth=[])) == []


def test_telemetry_audit_rejects_missing_sparse_dropped_and_misaligned_streams():
    from modular_robot_benchmarks.evaluator_metrics import telemetry_audit

    assert any("dropped 600" in p for p in telemetry_audit(_record(dropped=600)))
    assert any("no evaluator core truth" in p for p in telemetry_audit(_record(truth=[])))
    sparse = [{"time_s": 5.0 * i, "x": 0.0, "y": 0.0} for i in range(4)]
    assert any("localization history has 4" in p
               for p in telemetry_audit(_record(estimates=sparse)))
    early_truth = [_diagnostic(0.25 * i, 0.0, 0.0) for i in range(40)]
    problems = telemetry_audit(_record(truth=early_truth))
    assert any("synchronized truth" in p for p in problems)
    assert any("truth spans" in p for p in problems)
    assert any("do not match recomputation" in p
               for p in telemetry_audit(_record(errors=[0.0])))


def test_terminal_observation_snapshots_streams_before_recovery_probe():
    # The observer keeps spinning during the post-terminal recovery probe;
    # passing live lists let fault records absorb post-terminal samples.
    source = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/"
              "mission_trial.py").read_text(encoding="utf-8")
    observation = source.split("def _observation", 1)[1]
    for stream in ("covariance_trace", "topology_history", "execution_history",
                   "odometry_history", "command_history", "localization_history",
                   "pod_alignment_history"):
        assert f"list(node.{stream})" in observation
        assert f"=node.{stream}," not in observation


def test_autonomy_packages_never_reference_simulator_ground_truth():
    offenders = []
    for package in AUTONOMY_PACKAGES:
        for path in (ROOT / "src" / package).rglob("*"):
            if not path.is_file() or path.suffix not in {
                    ".py", ".cc", ".cpp", ".hpp", ".h", ".yaml", ".xml", ".xacro"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            offenders += [f"{path.relative_to(ROOT)}: {marker}"
                          for marker in GROUND_TRUTH_MARKERS if marker in text]
    assert offenders == []
