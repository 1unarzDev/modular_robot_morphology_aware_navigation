from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time

from .confirmatory_scenarios import make_confirmatory_scenario
from .design import FAULT_MATRIX, FAULT_MATRIX_DESIGN_KIND, StudyDesign
from .evaluator_metrics import localization_errors
from .records import TrialManifest, TrialRecord, TrialStore
from .sdf_export import export_confirmatory_sdf, export_occupancy_map


def classify_terminal(success: bool, message: str, timed_out: bool) -> str:
    if success:
        return "completed"
    if timed_out:
        return "timeout"
    lowered = message.lower()
    if "motion qualification failed" in lowered:
        return "motion_qualification_failure"
    if "costmap" in lowered and "acknowledge" in lowered:
        return "infrastructure_failure"
    if ("recovery required" in lowered or "requires recovery" in lowered
            or "unsafe topology" in lowered):
        return "unsafe_topology"
    if "reconfiguration" in lowered or "dock" in lowered or "transition" in lowered or "latch" in lowered:
        return "docking_failure"
    if "execution failed" in lowered or "controller" in lowered:
        return "controller_failure"
    if "plan" in lowered or "route" in lowered or "path" in lowered:
        return "planning_failure"
    if "localization" in lowered or "transform" in lowered:
        return "localization_lost"
    if "cancel" in lowered:
        return "cancelled"
    return "controller_failure"


def failure_injection_for(design: StudyDesign, spec) -> str:
    """Declared executor fault for a trial; empty outside the fault matrix."""
    if design.design_kind != FAULT_MATRIX_DESIGN_KIND:
        return ""
    return FAULT_MATRIX[spec.replicate].injection


def configuration_hash(paths: list[Path], parameters: dict) -> str:
    digest = hashlib.sha256()
    for path in sorted(paths, key=lambda value: str(value)):
        digest.update(str(path).encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    digest.update(json.dumps(parameters, sort_keys=True, separators=(",", ":")).encode())
    return digest.hexdigest()


def retryable_infrastructure_failure(status: str, launch_returncode: int | None) -> bool:
    """Only pre-mission infrastructure failures may receive bounded retries."""
    return status == "infrastructure_failure" and launch_returncode is None


def _git_revision(root: Path, allow_dirty: bool) -> str:
    revision = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain"], cwd=root, text=True).strip()
    if dirty and not allow_dirty:
        raise RuntimeError("refusing experiment run from a dirty worktree; use --allow-dirty for engineering runs")
    return revision + ("-dirty" if dirty else "")


def _stop_process(process: subprocess.Popen) -> None:
    descendants = _descendant_pids(process.pid)
    if process.poll() is not None:
        _terminate_pids(descendants)
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
    # gz_sim.launch starts a Ruby wrapper whose gz child can leave the launch
    # process group and be reparented to PID 1. Retain the pre-shutdown
    # descendant set so that this trial cannot contaminate the next one.
    _terminate_pids(descendants)


def _descendant_pids(root_pid: int) -> set[int]:
    parents: dict[int, list[int]] = {}
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            value = path.read_text(encoding="utf-8")
            closing = value.rfind(")")
            pid = int(value[:value.find(" ")])
            parent = int(value[closing + 2:].split()[1])
        except (OSError, ValueError, IndexError):
            continue
        parents.setdefault(parent, []).append(pid)
    descendants, frontier = set(), [root_pid]
    while frontier:
        parent = frontier.pop()
        for child in parents.get(parent, []):
            if child not in descendants:
                descendants.add(child)
                frontier.append(child)
    return descendants


def _terminate_pids(pids: set[int]) -> None:
    alive = set(pids)
    for requested_signal, timeout in (
            (signal.SIGTERM, 3.0), (signal.SIGKILL, 1.0)):
        for pid in tuple(alive):
            try:
                os.kill(pid, requested_signal)
            except ProcessLookupError:
                alive.discard(pid)
            except PermissionError:
                pass
        deadline = time.monotonic() + timeout
        while alive and time.monotonic() < deadline:
            alive = {pid for pid in alive if Path(f"/proc/{pid}").exists()}
            if alive:
                time.sleep(0.05)
        if not alive:
            return


def run_batch(
    root: Path,
    design_path: Path,
    raw_root: Path,
    artifact_root: Path,
    catalog_path: Path,
    max_trials: int | None,
    wall_watchdog_s: float,
    allow_dirty: bool,
    max_infrastructure_retries: int = 2,
) -> dict:
    # Import ROS only in an installed/runtime environment; pure design and
    # manifest helpers remain host-testable.
    from .mission_trial import MissionObservation, execute_mission

    design = StudyDesign.read_frozen(design_path)
    store = TrialStore(raw_root)
    pending = sorted(
        store.pending(design.trials),
        key=lambda spec: (spec.family, spec.layout_id, spec.replicate, spec.method_order),
    )
    if max_trials is not None:
        pending = pending[:max_trials]
    revision = _git_revision(root, allow_dirty)
    written = []
    for spec in pending:
        setup_start = time.monotonic()
        layout_index = int(spec.layout_id.rsplit("-", 1)[1])
        scenario = make_confirmatory_scenario(
            spec.family, layout_index, spec.world_seed)
        trial_artifacts = artifact_root / spec.trial_id
        trial_artifacts.mkdir(parents=True, exist_ok=True)
        world_path, scenario_manifest = export_confirmatory_sdf(
            spec.family, layout_index, spec.world_seed,
            trial_artifacts / "world.sdf", catalog_path, spec.friction_seed)
        map_yaml, map_image = export_occupancy_map(
            scenario.grid, trial_artifacts / "map.yaml")
        disturbance = json.loads(
            scenario_manifest.read_text(encoding="utf-8"))["physical_disturbance"]
        nominal_start_x, nominal_start_y = scenario.grid.cell_center(*scenario.start)
        start_x = nominal_start_x + disturbance["initial_dx_m"]
        start_y = nominal_start_y + disturbance["initial_dy_m"]
        start_yaw = disturbance["initial_yaw_rad"]
        parameters = {
            "deadline_s": 300.0,
            "wall_watchdog_s": wall_watchdog_s,
            "method": spec.method,
            "energy_measure": "10_newton_per_pod_absolute_speed_integral_proxy",
            "sequential_execution": True,
            "world_seed": spec.world_seed,
            "sensing_seed": spec.sensing_seed,
            "friction_seed": spec.friction_seed,
            "fault_seed": spec.fault_seed,
            "physical_disturbance": disturbance,
            "max_infrastructure_retries": max_infrastructure_retries,
        }
        failure_injection = failure_injection_for(design, spec)
        if failure_injection:
            parameters["failure_injection"] = failure_injection
            parameters["recovery_probe"] = True
        config_paths = [
            catalog_path,
            root / "src/modular_robot_bringup/config/nav2.yaml",
            root / "src/modular_robot_bringup/config/slam.yaml",
            root / "src/modular_robot_bringup/config/localization.yaml",
            world_path,
            scenario_manifest,
            map_yaml,
            map_image,
        ]
        manifest = TrialManifest(
            revision, configuration_hash(config_paths, parameters), design.design_hash,
            {
                "python": platform.python_version(),
                "platform": platform.platform(),
                "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
            },
            parameters,
        )
        setup_wall_s = time.monotonic() - setup_start
        infrastructure_attempts = []
        attempt_ledger_path = trial_artifacts / "infrastructure_attempts.jsonl"
        for attempt_index in range(max_infrastructure_retries + 1):
            prior_attempts = len(list(trial_artifacts.glob("launch*.log")))
            log_name = ("launch.log" if prior_attempts == 0
                        else f"launch.retry-{prior_attempts}.log")
            launch_log_path = trial_artifacts / log_name
            launch_log = launch_log_path.open("x", encoding="utf-8")
            process = subprocess.Popen(
                ["ros2", "launch", "modular_robot_bringup", "demo.launch.py",
                 f"world:={world_path.resolve()}", f"map:={map_yaml.resolve()}",
                 f"initial_x:={start_x}", f"initial_y:={start_y}",
                 f"initial_yaw:={start_yaw}",
                 *([f"failure_injection:={failure_injection}"] if failure_injection else [])],
                cwd=root, stdout=launch_log, stderr=subprocess.STDOUT,
                text=True, start_new_session=True,
            )
            try:
                goal = scenario.grid.cell_center(*scenario.goal)
                observation = execute_mission(
                    goal, spec.method, 300.0, wall_watchdog_s,
                    recovery_probe=bool(failure_injection))
                launch_returncode = process.poll()
                if launch_returncode is not None and not observation.completed:
                    observation.terminal_status = "process_crash"
                    observation.message += f"; launch exited {launch_returncode}"
            except Exception as exc:
                launch_returncode = process.poll()
                observation = MissionObservation(
                    "infrastructure_failure", False, f"runner exception: {exc}",
                    0.0, 0.0)
            finally:
                teardown_start = time.monotonic()
                _stop_process(process)
                launch_log.close()
                teardown_wall_s = time.monotonic() - teardown_start
            attempt_record = {
                "attempt": attempt_index + 1,
                "terminal_status": observation.terminal_status,
                "message": observation.message,
                "simulated_duration_s": observation.simulated_duration_s,
                "wall_duration_s": observation.wall_duration_s,
                "phase_timing": {**observation.phase_timing,
                                 "teardown_wall_s": teardown_wall_s},
                "launch_log": str(launch_log_path),
            }
            infrastructure_attempts.append(attempt_record)
            with attempt_ledger_path.open("a", encoding="utf-8") as ledger:
                ledger.write(json.dumps(attempt_record, sort_keys=True) + "\n")
                ledger.flush()
                os.fsync(ledger.fileno())
            if (attempt_index >= max_infrastructure_retries
                    or not retryable_infrastructure_failure(
                        observation.terminal_status, launch_returncode)):
                break
            time.sleep(2.0)
        record = TrialRecord(
            schema_version=1, spec=spec, manifest=manifest,
            terminal_status=observation.terminal_status,
            completed=observation.completed,
            simulated_duration_s=observation.simulated_duration_s,
            wall_duration_s=observation.wall_duration_s,
            planning_latency_s=observation.planning_latency_s,
            expanded_states=observation.expanded_states,
            mechanical_work_j=observation.mechanical_work_j,
            reconfiguration_attempts=observation.reconfiguration_attempts,
            reconfiguration_failures=observation.reconfiguration_failures,
            covariance_trace=observation.covariance_trace,
            odometry_history=observation.odometry_history,
            command_history=observation.command_history,
            planned_route=observation.planned_route,
            localization_history=observation.localization_history,
            localization_error_m=localization_errors(
                observation.localization_history,
                observation.controller_diagnostics.get("pod_drive_diagnostic_samples", [])),
            pod_alignment_history=observation.pod_alignment_history,
            motion_qualifications=observation.motion_qualifications,
            infrastructure_attempts=infrastructure_attempts,
            controller_diagnostics=observation.controller_diagnostics,
            topology_history=observation.topology_history,
            execution_state_history=observation.execution_state_history,
            final_execution_state=observation.final_execution_state,
            final_morphology=observation.final_morphology,
            unrecovered_fault=observation.unrecovered_fault,
            recovery_probe=observation.recovery_probe,
            phase_timing={**observation.phase_timing, "setup_wall_s": setup_wall_s,
                          "teardown_wall_s": teardown_wall_s},
            topology_revision=observation.topology_revision,
            sensing_revision=observation.sensing_revision,
            map_revision=observation.map_revision,
            planned_route_signature=observation.planned_route_signature,
            planned_transition_sites=observation.planned_transition_sites,
            notes=[observation.message, f"scenario_manifest={scenario_manifest}",
                   f"infrastructure_attempt_ledger={attempt_ledger_path}"],
        )
        store.write_terminal(record)
        written.append(spec.trial_id)
    return {
        "design_hash": design.design_hash,
        "written": written,
        "written_count": len(written),
        "remaining_count": len(store.pending(design.trials)),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run resumable ROS/Gazebo morphology missions")
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--design", type=Path, required=True)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--artifacts", type=Path, required=True)
    parser.add_argument(
        "--catalog", type=Path,
        default=Path("src/modular_robot_description/config/morphologies.yaml"))
    parser.add_argument("--max-trials", type=int)
    parser.add_argument("--wall-watchdog", type=float, default=600.0)
    parser.add_argument("--max-infrastructure-retries", type=int, default=2)
    parser.add_argument("--allow-dirty", action="store_true")
    return parser


def main() -> None:
    args = _parser().parse_args()
    result = run_batch(
        args.root.resolve(), args.design.resolve(), args.raw.resolve(),
        args.artifacts.resolve(), args.catalog.resolve(), args.max_trials,
        args.wall_watchdog, args.allow_dirty, args.max_infrastructure_retries)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
