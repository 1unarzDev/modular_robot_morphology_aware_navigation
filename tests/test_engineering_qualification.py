from dataclasses import replace
from math import cos, pi, sin
from pathlib import Path

import pytest

from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
from modular_robot_benchmarks.design import (
    FAULT_MATRIX, FAULT_MATRIX_DESIGN_KIND, ROUNDTRIP_DESIGN_KIND, StudyDesign,
    generate_engineering_design,
)
from modular_robot_benchmarks.engineering_qualification import (
    core_relative_pod_pose, evaluate_fault_trial, evaluate_roundtrip,
    load_morphology_pods, post_latch_rigidity, summarize_fault_matrix,
    summarize_roundtrips,
)
from modular_robot_benchmarks.mission_batch import failure_injection_for
from modular_robot_benchmarks.records import TrialManifest, TrialRecord, TrialStore
from reconfiguration_executor.control import injected_failure


ROOT = Path(__file__).parents[1]
PODS = load_morphology_pods(ROOT / "src/modular_robot_description/config/morphologies.yaml")
STAGES = {
    "positive_yaw": {"yaw_rad": 0.45, "forward_m": 0.0, "lateral_m": 0.0, "translation_m": 0.01},
    "negative_yaw": {"yaw_rad": -0.5, "forward_m": 0.0, "lateral_m": 0.0, "translation_m": 0.01},
    "forward": {"yaw_rad": 0.0, "forward_m": 0.1, "lateral_m": 0.002, "translation_m": 0.1},
    "reverse": {"yaw_rad": 0.0, "forward_m": -0.1, "lateral_m": -0.002, "translation_m": 0.1},
}
# (start, end, morphology) for READY windows around two transitions.
WINDOWS = ((0.0, 20.0, "compact_diff"), (40.0, 90.0, "narrow_tandem"),
           (110.0, 160.0, "compact_diff"))


def _world_sample(time_s, pod, relative, core):
    x, y, yaw = core
    rx, ry, ryaw = relative
    return {
        "time_s": time_s, "pod": pod,
        "world_x": x + cos(yaw) * rx - sin(yaw) * ry,
        "world_y": y + sin(yaw) * rx + cos(yaw) * ry,
        "world_z": 0.05, "world_roll": 0.0, "world_pitch": 0.0,
        "world_yaw": yaw + ryaw,
        "core_world_x": x, "core_world_y": y, "core_world_z": 0.23,
        "core_world_roll": 0.0, "core_world_pitch": 0.0, "core_world_yaw": yaw,
    }


def _samples(spec, drift=None):
    scenario = make_confirmatory_scenario(
        spec.family, int(spec.layout_id.rsplit("-", 1)[1]), spec.world_seed)
    start_x, start_y = scenario.grid.cell_center(*scenario.start)
    samples = []
    for start, end, morphology in WINDOWS:
        for step in range(int((end - start) * 2)):
            time_s = start + 0.5 * step
            core = (start_x + 0.01 * time_s, start_y, 0.002 * time_s)
            for pod, nominal in PODS[morphology].items():
                relative = list(nominal)
                if drift and drift[0] == morphology and drift[1] == pod:
                    relative[0] += drift[2] * step / max(1, int((end - start) * 2) - 1)
                samples.append(_world_sample(time_s, pod, relative, core))
        # Detached pods move freely while TRANSITIONING; never part of a window.
        # The final READY window is open-ended, so only earlier windows get one.
        if morphology != WINDOWS[-1][2] or start != WINDOWS[-1][0]:
            samples.append(_world_sample(end + 1.0, "pod_0", (3.0, 3.0, 1.0),
                                         (start_x, start_y, 0.0)))
    return samples


def _record(design, spec, tmp_path=None, friction=0.8, **changes):
    disturbance = {"ground_friction": friction, "initial_dx_m": 0.0,
                   "initial_dy_m": 0.0, "initial_yaw_rad": 0.0}
    if tmp_path is not None:
        world = tmp_path / spec.trial_id / "world.sdf"
        world.parent.mkdir(parents=True, exist_ok=True)
        world.write_text(
            "<sdf><world><model name='ground'><link name='link'><collision name='c'>"
            f"<surface><friction><ode><mu>{friction}</mu><mu2>{friction}</mu2></ode>"
            "</friction></surface></collision></link></model></world></sdf>")
    execution, topology = [], []
    for index, (start, end, morphology) in enumerate(WINDOWS):
        execution.append({"time_s": start, "state": "READY"})
        topology.append({"time_s": start, "revision": 1 + 12 * index,
                         "morphology": morphology, "graph_hash": morphology})
        if index < len(WINDOWS) - 1:
            execution.append({"time_s": end, "state": "TRANSITIONING"})
    record = TrialRecord(
        schema_version=1, spec=spec,
        manifest=TrialManifest("abc", "cfg", design.design_hash, {},
                               {"physical_disturbance": disturbance}),
        terminal_status="completed", completed=True, simulated_duration_s=160.0,
        wall_duration_s=320.0, reconfiguration_attempts=2,
        motion_qualifications=[{"passed": True, "stages": STAGES}] * 2,
        infrastructure_attempts=[{"attempt": 1, "terminal_status": "completed"}],
        controller_diagnostics={"pod_drive_diagnostic_samples": _samples(spec)},
        topology_history=topology, execution_state_history=execution,
        final_execution_state="READY", final_morphology="compact_diff",
    )
    return replace(record, **changes)


@pytest.fixture
def roundtrip_design():
    return generate_engineering_design(
        ROUNDTRIP_DESIGN_KIND, "reconfiguration_workspace", 1,
        "sensing_feasibility_coupled", runs=3)


def test_core_relative_pose_inverts_rotated_planar_composition():
    sample = _world_sample(0.0, "pod_4", (0.35, 0.20, -pi / 2), (2.0, -1.0, 3.0))
    pose = core_relative_pod_pose(sample)
    assert pose["x"] == pytest.approx(0.35)
    assert pose["y"] == pytest.approx(0.20)
    assert pose["yaw"] == pytest.approx(-pi / 2)


def test_rigidity_covers_every_ready_window_and_ignores_transitions(roundtrip_design):
    spec = roundtrip_design.trials[0]
    rigidity = post_latch_rigidity(_record(roundtrip_design, spec), PODS)
    assert rigidity["passed"]
    assert [window["morphology"] for window in rigidity["windows"]] == [
        "compact_diff", "narrow_tandem", "compact_diff"]
    assert all(pod["max_position_error_m"] < 1e-9
               for window in rigidity["windows"] for pod in window["pods"].values())


def test_rigidity_fails_when_a_latched_pod_drifts_beyond_capture_tolerance(roundtrip_design):
    spec = roundtrip_design.trials[0]
    record = _record(roundtrip_design, spec, controller_diagnostics={
        "pod_drive_diagnostic_samples": _samples(spec, ("narrow_tandem", "pod_3", 0.02))})
    rigidity = post_latch_rigidity(record, PODS)
    narrow = rigidity["windows"][1]
    assert not rigidity["passed"] and not narrow["passed"]
    assert [pod for pod, value in narrow["pods"].items() if not value["passed"]] == ["pod_3"]
    assert narrow["pods"]["pod_3"]["position_spread_m"] > 0.009


def test_rigidity_requires_core_synchronized_samples_for_every_pod(roundtrip_design):
    spec = roundtrip_design.trials[0]
    samples = [sample for sample in _samples(spec) if sample["pod"] != "pod_5"]
    samples += [{key: value for key, value in sample.items()
                 if not key.startswith("core_")}
                for sample in _samples(spec) if sample["pod"] == "pod_5"]
    record = _record(roundtrip_design, spec,
                     controller_diagnostics={"pod_drive_diagnostic_samples": samples})
    rigidity = post_latch_rigidity(record, PODS)
    assert not rigidity["passed"]
    assert rigidity["windows"][0]["pods"]["pod_5"]["reason"] == "insufficient_samples"


def test_roundtrip_contract_names_each_failed_requirement(roundtrip_design, tmp_path):
    spec = roundtrip_design.trials[0]
    passing = evaluate_roundtrip(_record(roundtrip_design, spec, tmp_path), PODS, tmp_path)
    assert passing["passed"], passing["failed_checks"]
    one_gate = _record(roundtrip_design, spec, tmp_path,
                       motion_qualifications=[{"passed": True, "stages": STAGES}])
    assert evaluate_roundtrip(one_gate, PODS, tmp_path)["failed_checks"] == [
        "motion_gates_passed"]
    stranded = _record(
        roundtrip_design, spec, tmp_path, terminal_status="unsafe_topology",
        completed=False, final_execution_state="RECOVERY_REQUIRED",
        final_morphology="narrow_tandem", unrecovered_fault=True)
    assert {"completed", "final_compact_ready", "no_unrecovered_fault"} <= set(
        evaluate_roundtrip(stranded, PODS, tmp_path)["failed_checks"])
    assert "disturbance_applied" in evaluate_roundtrip(
        _record(roundtrip_design, spec), PODS, tmp_path / "missing")["failed_checks"]


def test_roundtrip_gate_requires_consecutive_passes_and_unique_plants(
        roundtrip_design, tmp_path):
    specs = roundtrip_design.trials
    records = [_record(roundtrip_design, spec, tmp_path, friction=0.75 + 0.1 * index)
               for index, spec in enumerate(specs)]
    summary = summarize_roundtrips(roundtrip_design, records, PODS, tmp_path)
    assert summary["gate_passed"] and summary["consecutive_passes_from_first_run"] == 3
    assert summary["realized_plant_variation"]["sdf_ground_friction"]["count"] == 3
    assert summary["realized_plant_variation"]["sdf_ground_friction"]["sd"] > 0

    duplicate_root = tmp_path / "duplicate"
    duplicated = [_record(roundtrip_design, spec, duplicate_root, friction=0.75)
                  for spec in specs]
    duplicate_summary = summarize_roundtrips(
        roundtrip_design, duplicated, PODS, duplicate_root)
    assert not duplicate_summary["unique_realized_disturbances"]
    assert not duplicate_summary["gate_passed"]

    missing_middle = summarize_roundtrips(
        roundtrip_design, [records[0], records[2]], PODS, tmp_path)
    assert not missing_middle["gate_passed"]
    assert missing_middle["consecutive_passes_from_first_run"] == 1
    assert missing_middle["failed_check_counts"] == {"pending": 1}


def test_engineering_designs_freeze_with_one_method_and_independent_plant_seeds(tmp_path):
    design = generate_engineering_design(
        ROUNDTRIP_DESIGN_KIND, "combined_constraints", 5, "geometry_coupled")
    assert design.expected_trials == 20 and design.methods == ("geometry_coupled",)
    assert len({spec.world_seed for spec in design.trials}) == 1
    assert len({spec.friction_seed for spec in design.trials}) == 20
    path = design.write_frozen(tmp_path / "roundtrip.json")
    assert StudyDesign.read_frozen(path) == design
    faults = generate_engineering_design(
        FAULT_MATRIX_DESIGN_KIND, "combined_constraints", 5, "geometry_coupled")
    assert faults.expected_trials == len(FAULT_MATRIX)
    with pytest.raises(ValueError, match="fault count"):
        generate_engineering_design(
            FAULT_MATRIX_DESIGN_KIND, "combined_constraints", 5, "geometry_coupled", runs=3)


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
    design = generate_engineering_design(
        FAULT_MATRIX_DESIGN_KIND, "combined_constraints", 5, "geometry_coupled")
    assert [failure_injection_for(design, spec) for spec in design.trials] == [
        case.injection for case in FAULT_MATRIX]
    roundtrip = generate_engineering_design(
        ROUNDTRIP_DESIGN_KIND, "combined_constraints", 5, "geometry_coupled", runs=1)
    assert failure_injection_for(roundtrip, roundtrip.trials[0]) == ""


def _fault_record(design, spec, probe):
    case = FAULT_MATRIX[spec.replicate]
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
    design = generate_engineering_design(
        FAULT_MATRIX_DESIGN_KIND, "combined_constraints", 5, "geometry_coupled")
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
    store.write_terminal(moved)
    assert store.load_all()[0].recovery_probe["max_pod_command"] == 0.1


def test_demo_launch_forwards_declared_failure_injection_to_executor():
    launch = (ROOT / "src/modular_robot_bringup/launch/demo.launch.py").read_text()
    assert 'DeclareLaunchArgument(\n            "failure_injection"' in launch
    assert '"failure_injection": ParameterValue(failure_injection, value_type=str)' in launch
