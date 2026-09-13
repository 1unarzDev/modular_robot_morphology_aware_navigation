"""Gate 0 engineering qualification: randomized round trips and fault matrix.

Evaluator-only Gazebo poses are consumed here after a trial has ended. Nothing
in this module is reachable from localization, planning, docking, or recovery.
Engineering records are never pilot or confirmatory evidence.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from math import atan2, cos, hypot, pi, sin
from pathlib import Path
from statistics import mean, pstdev
from typing import Any, Iterable, Mapping
from xml.etree import ElementTree

import yaml

from .confirmatory_scenarios import make_confirmatory_scenario
from .design import (
    FAULT_MATRIX, FAULT_MATRIX_DESIGN_KIND, ROUNDTRIP_DESIGN_KIND, FaultCase,
    StudyDesign, generate_engineering_design,
)
from .records import TrialRecord, TrialStore


ROUNDTRIP_MORPHOLOGIES = ("compact_diff", "narrow_tandem", "compact_diff")
# Frozen before the 20-run gate: the executor's connector capture envelope,
# applied to evaluator pod poses throughout every READY window.
RIGIDITY_POSITION_TOLERANCE_M = 0.012
RIGIDITY_YAW_TOLERANCE_RAD = 0.05236
# Execution-state events are stamped on receipt. Exclude samples just before
# the next state change so a physical detach cannot enter a READY window.
READY_WINDOW_END_GUARD_S = 0.2
MIN_RIGIDITY_SAMPLES_PER_POD = 2
# Assembled drive is inhibited when no pod receives motion and the core stays put.
INHIBITION_MAX_POD_COMMAND = 1e-6
INHIBITION_MAX_CORE_TRANSLATION_M = 0.01
INHIBITION_MAX_CORE_YAW_RAD = 0.02
CORE_POSE_KEYS = tuple(
    f"core_world_{axis}" for axis in ("x", "y", "z", "roll", "pitch", "yaw"))


def _wrap(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def _rotation(roll: float, pitch: float, yaw: float) -> tuple[tuple[float, ...], ...]:
    """Gazebo/SDF extrinsic XYZ Euler angles as a body-to-world matrix."""
    cr, sr, cp, sp, cy, sy = cos(roll), sin(roll), cos(pitch), sin(pitch), cos(yaw), sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _has_core_pose(sample: Mapping[str, Any]) -> bool:
    return all(key in sample for key in CORE_POSE_KEYS)


def core_relative_pod_pose(sample: Mapping[str, Any]) -> dict[str, float]:
    """Express one evaluator pod pose in the core base-link frame."""
    core = _rotation(sample["core_world_roll"], sample["core_world_pitch"],
                     sample["core_world_yaw"])
    pod = _rotation(sample.get("world_roll", 0.0), sample.get("world_pitch", 0.0),
                    sample["world_yaw"])
    delta = (sample["world_x"] - sample["core_world_x"],
             sample["world_y"] - sample["core_world_y"],
             sample.get("world_z", 0.0) - sample["core_world_z"])
    local = [sum(core[row][axis] * delta[row] for row in range(3)) for axis in range(3)]
    relative = [[sum(core[k][i] * pod[k][j] for k in range(3)) for j in range(3)]
                for i in range(3)]
    return {"x": local[0], "y": local[1], "z": local[2],
            "yaw": atan2(relative[1][0], relative[0][0])}


def ready_windows(
    execution_history: list[Mapping[str, Any]],
    topology_history: list[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """READY intervals with the morphology acknowledged when each began."""
    windows = []
    for index, event in enumerate(execution_history):
        if event["state"] != "READY":
            continue
        start = float(event["time_s"])
        following = execution_history[index + 1:]
        known = [entry for entry in topology_history if float(entry["time_s"]) <= start]
        morphology = (known[-1] if known else (topology_history or [{}])[0]).get("morphology", "")
        windows.append({
            "start_s": start,
            "end_s": float(following[0]["time_s"]) if following else None,
            "morphology": morphology,
        })
    return windows


def post_latch_rigidity(
    record: TrialRecord, morphology_pods: Mapping[str, Mapping[str, list[float]]],
) -> dict[str, Any]:
    """Per-pod core-relative pose error against the catalog in every READY window."""
    samples = [sample for sample in record.controller_diagnostics.get(
        "pod_drive_diagnostic_samples", []) if _has_core_pose(sample)]
    windows = []
    for window in ready_windows(record.execution_state_history, record.topology_history):
        nominal_pods = morphology_pods.get(window["morphology"], {})
        end = (window["end_s"] - READY_WINDOW_END_GUARD_S
               if window["end_s"] is not None else float("inf"))
        pods = {}
        for pod, nominal in sorted(nominal_pods.items()):
            poses = [core_relative_pod_pose(sample) for sample in samples
                     if sample["pod"] == pod and window["start_s"] <= sample["time_s"] < end]
            if len(poses) < MIN_RIGIDITY_SAMPLES_PER_POD:
                pods[pod] = {"sample_count": len(poses), "passed": False,
                             "reason": "insufficient_samples"}
                continue
            mean_x = mean(pose["x"] for pose in poses)
            mean_y = mean(pose["y"] for pose in poses)
            position_errors = [hypot(pose["x"] - nominal[0], pose["y"] - nominal[1])
                               for pose in poses]
            yaw_errors = [abs(_wrap(pose["yaw"] - nominal[2])) for pose in poses]
            pods[pod] = {
                "sample_count": len(poses),
                "max_position_error_m": max(position_errors),
                "max_yaw_error_rad": max(yaw_errors),
                "position_spread_m": max(hypot(pose["x"] - mean_x, pose["y"] - mean_y)
                                         for pose in poses),
                "z_spread_m": max(pose["z"] for pose in poses) - min(pose["z"] for pose in poses),
                "passed": (max(position_errors) <= RIGIDITY_POSITION_TOLERANCE_M
                           and max(yaw_errors) <= RIGIDITY_YAW_TOLERANCE_RAD),
            }
        windows.append({
            **window, "pods": pods,
            "passed": bool(pods) and all(value["passed"] for value in pods.values()),
        })
    return {
        "position_tolerance_m": RIGIDITY_POSITION_TOLERANCE_M,
        "yaw_tolerance_rad": RIGIDITY_YAW_TOLERANCE_RAD,
        "samples_dropped": int(record.controller_diagnostics.get(
            "pod_drive_diagnostic_samples_dropped", 0)),
        "windows": windows,
        "passed": bool(windows) and all(window["passed"] for window in windows),
    }


def motion_gate_summary(qualifications: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for index, qualification in enumerate(qualifications):
        stages = qualification.get("stages") or {}
        rows.append({
            "index": index,
            "passed": qualification.get("passed") is True,
            **{name: {key: (stages.get(name) or {}).get(key)
                      for key in ("yaw_rad", "forward_m", "lateral_m", "translation_m")}
               for name in ("positive_yaw", "negative_yaw", "forward", "reverse")},
        })
    return rows


def _ground_friction(world: Path) -> float | None:
    root = ElementTree.parse(world).getroot()
    for model in root.iter("model"):
        if model.get("name") == "ground":
            value = model.find(".//friction/ode/mu")
            return float(value.text) if value is not None else None
    return None


def realized_plant(record: TrialRecord, artifact_root: Path | None) -> dict[str, Any]:
    """Plant nuisance values read from the loaded world and measured at start."""
    declared = dict(record.manifest.parameters.get("physical_disturbance", {}))
    world = (Path(artifact_root) / record.spec.trial_id / "world.sdf"
             if artifact_root is not None else None)
    friction = _ground_friction(world) if world is not None and world.is_file() else None
    samples = record.controller_diagnostics.get("pod_drive_diagnostic_samples", [])
    first = next((sample for sample in samples if _has_core_pose(sample)), None)
    measured = None
    if first is not None:
        scenario = make_confirmatory_scenario(
            record.spec.family, int(record.spec.layout_id.rsplit("-", 1)[1]),
            record.spec.world_seed)
        nominal_x, nominal_y = scenario.grid.cell_center(*scenario.start)
        measured = {
            "time_s": float(first["time_s"]),
            "initial_dx_m": float(first["core_world_x"]) - nominal_x,
            "initial_dy_m": float(first["core_world_y"]) - nominal_y,
            "initial_yaw_rad": _wrap(float(first["core_world_yaw"])),
        }
    return {"declared": declared, "sdf_ground_friction": friction,
            "measured_initial_offset": measured}


def _dedupe(values: Iterable[str]) -> list[str]:
    result: list[str] = []
    for value in values:
        if not result or result[-1] != value:
            result.append(value)
    return result


def evaluate_roundtrip(
    record: TrialRecord, morphology_pods: Mapping[str, Mapping[str, list[float]]],
    artifact_root: Path | None = None,
) -> dict[str, Any]:
    rigidity = post_latch_rigidity(record, morphology_pods)
    plant = realized_plant(record, artifact_root)
    declared_friction = plant["declared"].get("ground_friction")
    states = [event["state"] for event in record.execution_state_history]
    checks = {
        "completed": record.completed,
        "two_successful_transitions": (
            record.reconfiguration_attempts == 2
            and _dedupe(entry["morphology"] for entry in record.topology_history)
            == list(ROUNDTRIP_MORPHOLOGIES)),
        "morphology_acknowledged": (
            [window["morphology"] for window in rigidity["windows"]]
            == list(ROUNDTRIP_MORPHOLOGIES)),
        "final_compact_ready": (record.final_morphology == "compact_diff"
                                and record.final_execution_state == "READY"),
        "no_unrecovered_fault": (not record.unrecovered_fault
                                 and "RECOVERY_REQUIRED" not in states),
        "motion_gates_passed": (len(record.motion_qualifications) == 2 and all(
            value.get("passed") is True for value in record.motion_qualifications)),
        "post_latch_rigidity": rigidity["passed"],
        "disturbance_applied": (
            plant["sdf_ground_friction"] is not None and declared_friction is not None
            and abs(plant["sdf_ground_friction"] - declared_friction) <= 1e-9
            and plant["measured_initial_offset"] is not None),
    }
    return {
        "trial_id": record.spec.trial_id,
        "replicate": record.spec.replicate,
        "terminal_status": record.terminal_status,
        "simulated_duration_s": record.simulated_duration_s,
        "passed": all(checks.values()),
        "checks": checks,
        "failed_checks": sorted(name for name, value in checks.items() if not value),
        "motion_gates": motion_gate_summary(record.motion_qualifications),
        "rigidity": rigidity,
        "realized_plant": plant,
    }


def _stats(values: Iterable[float | None]) -> dict[str, float | int]:
    present = [float(value) for value in values if value is not None]
    if not present:
        return {"count": 0}
    return {"count": len(present), "min": min(present), "max": max(present),
            "mean": mean(present), "sd": pstdev(present)}


def _disturbance_key(plant: Mapping[str, Any]) -> tuple:
    declared = plant["declared"]
    friction = plant["sdf_ground_friction"]
    return tuple(round(float(value), 9) if value is not None else None for value in (
        friction if friction is not None else declared.get("ground_friction"),
        declared.get("initial_dx_m"), declared.get("initial_dy_m"),
        declared.get("initial_yaw_rad")))


def _checked_records(design: StudyDesign, records: list[TrialRecord], kind: str):
    if design.design_kind != kind:
        raise ValueError(f"design kind {design.design_kind} is not {kind}")
    expected = {spec.trial_id for spec in design.trials}
    wrong_hash = sorted(record.spec.trial_id for record in records
                        if record.manifest.design_hash != design.design_hash)
    if wrong_hash:
        raise ValueError(f"records have a different design hash: {wrong_hash}")
    unexpected = sorted(record.spec.trial_id for record in records
                        if record.spec.trial_id not in expected)
    return {record.spec.trial_id: record for record in records}, unexpected


def summarize_roundtrips(
    design: StudyDesign, records: list[TrialRecord],
    morphology_pods: Mapping[str, Mapping[str, list[float]]],
    artifact_root: Path | None = None,
) -> dict[str, Any]:
    by_id, unexpected = _checked_records(design, records, ROUNDTRIP_DESIGN_KIND)
    runs, consecutive, streak_open = [], 0, True
    for spec in sorted(design.trials, key=lambda value: value.replicate):
        record = by_id.get(spec.trial_id)
        if record is None:
            runs.append({"trial_id": spec.trial_id, "replicate": spec.replicate,
                         "passed": False, "failed_checks": ["pending"]})
            streak_open = False
            continue
        evaluation = evaluate_roundtrip(record, morphology_pods, artifact_root)
        runs.append(evaluation)
        if evaluation["passed"] and streak_open:
            consecutive += 1
        else:
            streak_open = False
    evaluated = [run for run in runs if "realized_plant" in run]
    keys = [_disturbance_key(run["realized_plant"]) for run in evaluated]
    unique_disturbances = len(set(keys)) == len(keys)

    def gate_values(gate: int, stage: str, key: str):
        return [run["motion_gates"][gate][stage][key] for run in evaluated
                if len(run["motion_gates"]) > gate]

    rigidity_errors = [
        (pod["max_position_error_m"], pod["max_yaw_error_rad"])
        for run in evaluated for window in run["rigidity"]["windows"]
        for pod in window["pods"].values() if "max_position_error_m" in pod]
    return {
        "purpose": "engineering_qualification_not_study_evidence",
        "design_hash": design.design_hash,
        "expected_runs": design.expected_trials,
        "observed_runs": len(evaluated),
        "unexpected_records": unexpected,
        "consecutive_passes_from_first_run": consecutive,
        "unique_realized_disturbances": unique_disturbances,
        "gate_passed": consecutive == design.expected_trials and unique_disturbances,
        "failed_check_counts": dict(sorted(Counter(
            name for run in runs for name in run["failed_checks"]).items())),
        "terminal_status_counts": dict(sorted(Counter(
            run["terminal_status"] for run in evaluated).items())),
        "realized_plant_variation": {
            "sdf_ground_friction": _stats(
                run["realized_plant"]["sdf_ground_friction"] for run in evaluated),
            **{f"measured_{key}": _stats(
                (run["realized_plant"]["measured_initial_offset"] or {}).get(key)
                for run in evaluated)
               for key in ("initial_dx_m", "initial_dy_m", "initial_yaw_rad")},
        },
        "motion_gate_variation": {
            f"gate_{gate}": {
                "positive_yaw_rad": _stats(gate_values(gate, "positive_yaw", "yaw_rad")),
                "negative_yaw_rad": _stats(gate_values(gate, "negative_yaw", "yaw_rad")),
                "forward_m": _stats(gate_values(gate, "forward", "forward_m")),
                "reverse_m": _stats(gate_values(gate, "reverse", "forward_m")),
                "forward_lateral_m": _stats(gate_values(gate, "forward", "lateral_m")),
                "reverse_lateral_m": _stats(gate_values(gate, "reverse", "lateral_m")),
            } for gate in range(2)
        },
        "rigidity_variation": {
            "max_position_error_m": _stats(error[0] for error in rigidity_errors),
            "max_yaw_error_rad": _stats(error[1] for error in rigidity_errors),
        },
        "runs": runs,
    }


def evaluate_fault_trial(record: TrialRecord, case: FaultCase) -> dict[str, Any]:
    probe = record.recovery_probe
    reconcile = probe.get("reconcile") or {}
    if case.expected_reconciled_morphology is None:
        reconciliation = (reconcile.get("success") is False
                          and reconcile.get("execution_state") == "RECOVERY_REQUIRED")
    else:
        reconciliation = (reconcile.get("success") is True
                          and reconcile.get("execution_state") == "READY"
                          and reconcile.get("morphology") == case.expected_reconciled_morphology)
    checks = {
        "declared_injection": (
            record.manifest.parameters.get("failure_injection") == case.injection),
        "fail_closed_terminal": (not record.completed
                                 and record.terminal_status == "unsafe_topology"),
        "recovery_required": (probe.get("state_before") == "RECOVERY_REQUIRED"
                              and record.final_execution_state == "RECOVERY_REQUIRED"),
        "drive_inhibited": (
            probe.get("performed") is True
            and probe.get("max_pod_command", float("inf")) <= INHIBITION_MAX_POD_COMMAND
            and probe.get("core_translation_m", float("inf"))
            <= INHIBITION_MAX_CORE_TRANSLATION_M
            and probe.get("core_yaw_change_rad", float("inf")) <= INHIBITION_MAX_CORE_YAW_RAD),
        "reconciliation_matches_observed_topology": reconciliation,
    }
    return {
        "case": case.name, "injection": case.injection,
        "expected_reconciled_morphology": case.expected_reconciled_morphology,
        "trial_id": record.spec.trial_id, "terminal_status": record.terminal_status,
        "message": record.notes[0] if record.notes else "",
        "passed": all(checks.values()), "checks": checks,
        "failed_checks": sorted(name for name, value in checks.items() if not value),
        "recovery_probe": probe,
    }


def summarize_fault_matrix(design: StudyDesign, records: list[TrialRecord]) -> dict[str, Any]:
    by_id, unexpected = _checked_records(design, records, FAULT_MATRIX_DESIGN_KIND)
    cases = []
    for spec in sorted(design.trials, key=lambda value: value.replicate):
        case = FAULT_MATRIX[spec.replicate]
        record = by_id.get(spec.trial_id)
        cases.append(evaluate_fault_trial(record, case) if record is not None else {
            "case": case.name, "injection": case.injection, "trial_id": spec.trial_id,
            "passed": False, "failed_checks": ["pending"]})
    return {
        "purpose": "engineering_qualification_not_study_evidence",
        "design_hash": design.design_hash,
        "unexpected_records": unexpected,
        "gate_passed": all(case["passed"] for case in cases),
        "cases": cases,
    }


def load_morphology_pods(catalog_path: Path) -> dict[str, dict[str, list[float]]]:
    with Path(catalog_path).open(encoding="utf-8") as stream:
        catalog = yaml.safe_load(stream)
    return {name: value["pods"] for name, value in catalog["morphologies"].items()}


def _write_json(value: Mapping[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Freeze and evaluate Gate 0 engineering qualification runs")
    commands = parser.add_subparsers(dest="command", required=True)
    for name, kind in (("freeze-roundtrip", ROUNDTRIP_DESIGN_KIND),
                       ("freeze-fault-matrix", FAULT_MATRIX_DESIGN_KIND)):
        freeze = commands.add_parser(name, help=f"write an immutable {kind} design")
        freeze.set_defaults(kind=kind)
        freeze.add_argument("--output", type=Path, required=True)
        freeze.add_argument("--family", required=True)
        freeze.add_argument("--layout-index", type=int, required=True)
        freeze.add_argument("--method", required=True)
        freeze.add_argument("--master-seed", type=int, default=20260912)
        if kind == ROUNDTRIP_DESIGN_KIND:
            freeze.add_argument("--runs", type=int, default=20)
    for name in ("roundtrip", "fault-matrix"):
        summary = commands.add_parser(name, help=f"summarize {name} records")
        summary.add_argument("--design", type=Path, required=True)
        summary.add_argument("--raw", type=Path, required=True)
        summary.add_argument("--output", type=Path, required=True)
        if name == "roundtrip":
            summary.add_argument("--artifacts", type=Path, required=True)
            summary.add_argument(
                "--catalog", type=Path,
                default=Path("src/modular_robot_description/config/morphologies.yaml"))
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.command.startswith("freeze-"):
        design = generate_engineering_design(
            args.kind, args.family, args.layout_index, args.method,
            getattr(args, "runs", None), args.master_seed)
        design.write_frozen(args.output)
        result = {"path": str(args.output), "design_hash": design.design_hash,
                  "expected_trials": design.expected_trials}
    else:
        design = StudyDesign.read_frozen(args.design)
        records = TrialStore(args.raw).load_all()
        if args.command == "roundtrip":
            summary = summarize_roundtrips(
                design, records, load_morphology_pods(args.catalog), args.artifacts)
        else:
            summary = summarize_fault_matrix(design, records)
        _write_json(summary, args.output)
        result = {key: summary[key] for key in ("design_hash", "gate_passed")}
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
