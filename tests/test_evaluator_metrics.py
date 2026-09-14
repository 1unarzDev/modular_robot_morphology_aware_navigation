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


def test_box_clearance_measures_planar_vertical_and_contact_gaps():
    from math import pi
    from modular_robot_benchmarks.evaluator_metrics import box_clearance

    wall = ((1.0, 0.0, 0.5), (0.1, 1.0, 1.0))
    # 0.2 m wide module centered 0.4 m before the wall face at x=0.95.
    assert box_clearance((0.45, 0.0, 0.1), (0.2, 0.2, 0.1), 0.0, *wall) == pytest.approx(0.4)
    # Rotating a square by 45 degrees brings its corner 0.0414 m closer.
    assert box_clearance((0.45, 0.0, 0.1), (0.2, 0.2, 0.1), pi / 4, *wall) == pytest.approx(
        0.4 - (0.2 * 2 ** 0.5 / 2 - 0.1), abs=1e-9)
    assert box_clearance((0.92, 0.0, 0.1), (0.2, 0.2, 0.1), 0.0, *wall) == 0.0
    shelf = ((0.0, 0.0, 0.14), (1.0, 0.16, 0.08))
    # Planar overlap under a raised shelf leaves only the vertical gap.
    assert box_clearance((0.0, 0.0, 0.03), (0.2, 0.2, 0.06), 0.0, *shelf) == pytest.approx(0.04)


def test_clearance_metrics_count_contact_events_per_module():
    from modular_robot_benchmarks.evaluator_metrics import clearance_metrics

    wall = [((1.0, 0.0, 0.5), (0.1, 1.0, 1.0))]
    def pod(time_s, x):
        return {"time_s": time_s, "module": "pod_0", "x": x, "y": 0.0, "z": 0.0, "yaw": 0.0}
    samples = [pod(0.0, 0.5), pod(0.1, 0.87), pod(0.2, 0.87), pod(2.0, 0.5), pod(3.0, 0.87)]
    metrics = clearance_metrics(samples, wall)
    assert metrics["collision_count"] == 2
    assert metrics["minimum_clearance_m"] == 0.0
    assert metrics["minimum_clearance_at"] == {"time_s": 0.1, "module": "pod_0"}
    clear = clearance_metrics([pod(0.0, 0.5)], wall)
    assert clear["collision_count"] == 0 and clear["minimum_clearance_m"] == pytest.approx(0.36)
    assert clearance_metrics([pod(0.0, -5.0)], wall)["minimum_clearance_m"] is None


def test_batch_records_evaluator_contact_and_reclassifies_completed_collisions():
    batch = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/"
             "mission_batch.py").read_text(encoding="utf-8")
    loop = batch.split("teardown_wall_s = time.monotonic() - teardown_start", 1)[1]
    loop = loop.split("attempt_record = {", 1)[0]
    assert "clearance_metrics(" in loop
    assert 'observation.terminal_status, observation.completed = "collision", False' in loop
    assert 'collision_count=clearance["collision_count"]' in batch
    assert 'minimum_clearance_m=clearance["minimum_clearance_m"]' in batch


def test_transition_attempts_become_success_probability_calibration_pairs():
    from modular_robot_benchmarks.evaluator_metrics import transition_calibration_pairs

    attempts = [
        {"transition_id": "compact_to_narrow", "predicted_failure_probability": 0.1,
         "success": True},
        {"transition_id": "narrow_to_compact", "predicted_failure_probability": 0.25,
         "success": False},
    ]
    predictions, outcomes = transition_calibration_pairs(attempts)
    assert predictions == pytest.approx([0.9, 0.75])
    assert outcomes == [1, 0]
    with pytest.raises(ValueError, match="out of range"):
        transition_calibration_pairs([{"predicted_failure_probability": 1.5, "success": True}])


def test_navigator_reports_every_transition_attempt_with_pre_action_risk():
    source = (ROOT / "src/modular_robot_bringup/modular_robot_bringup/"
              "hybrid_navigator.py").read_text(encoding="utf-8")
    branch = source.split("success = await self._execute_transition(segment)", 1)[1]
    branch = branch.split("qualification_passed = True", 1)[0]
    assert "result.transition_attempt_json" in branch
    assert "segment.predicted_failure_probability" in branch
    action = (ROOT / "src/modular_robot_msgs/action/NavigateHybrid.action").read_text(
        encoding="utf-8")
    assert "string[] transition_attempt_json" in action.split("---")[1]


def test_edge_decisions_aggregate_by_outcome_and_weight_analysis_counts():
    from morphology_planner.methods import aggregate_decision_records

    decisions = [
        {"method": "sensing_feasibility_coupled", "transition_id": "compact_to_narrow",
         "feasible": False, "reasons": ["pod_1:connector_not_visible"], "state": {}},
        {"method": "sensing_feasibility_coupled", "transition_id": "compact_to_narrow",
         "feasible": False, "reasons": ["pod_1:connector_not_visible"], "state": {}},
        {"method": "sensing_feasibility_coupled", "transition_id": "compact_to_narrow",
         "feasible": True, "reasons": [], "state": {}},
    ]
    aggregated = aggregate_decision_records(decisions)
    assert [(row["feasible"], row["count"]) for row in aggregated] == [(False, 2), (True, 1)]
    assert all({"transition_id", "feasible", "reasons"} <= row.keys() for row in aggregated)
    analysis = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/"
                "analysis.py").read_text(encoding="utf-8")
    assert 'int(decision.get("count", 1))' in analysis


def test_planner_and_navigator_forward_aggregated_edge_decisions():
    planner = (ROOT / "src/morphology_planner/morphology_planner/ros_node.py").read_text(
        encoding="utf-8")
    assert planner.count("result.transition_decision_json") >= 2  # success and no-path
    navigator = (ROOT / "src/modular_robot_bringup/modular_robot_bringup/"
                 "hybrid_navigator.py").read_text(encoding="utf-8")
    assert "*plan_result.transition_decision_json" in navigator
    for action in ("ComputeHybridPlan", "NavigateHybrid"):
        text = (ROOT / f"src/modular_robot_msgs/action/{action}.action").read_text(
            encoding="utf-8")
        assert "string[] transition_decision_json" in text.split("---")[1]


def test_recovery_actions_count_autonomous_replans_after_failed_execution():
    navigator = (ROOT / "src/modular_robot_bringup/modular_robot_bringup/"
                 "hybrid_navigator.py").read_text(encoding="utf-8")
    assert "execution_failures += 1\n            result.recovery_actions = execution_failures" in navigator
    action = (ROOT / "src/modular_robot_msgs/action/NavigateHybrid.action").read_text(
        encoding="utf-8")
    assert "uint32 recovery_actions" in action.split("---")[1]
    batch = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/"
             "mission_batch.py").read_text(encoding="utf-8")
    assert "recovery_actions=observation.recovery_actions" in batch


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
