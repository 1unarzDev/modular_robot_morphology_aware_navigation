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
from .design import StudyDesign
from .records import TrialManifest, TrialRecord, TrialStore
from .sdf_export import export_confirmatory_sdf, export_occupancy_map


def classify_terminal(success: bool, message: str, timed_out: bool) -> str:
    if success:
        return "completed"
    if timed_out:
        return "timeout"
    lowered = message.lower()
    if "execution failed" in lowered or "controller" in lowered:
        return "controller_failure"
    if "plan" in lowered or "route" in lowered or "path" in lowered:
        return "planning_failure"
    if "localization" in lowered or "transform" in lowered:
        return "localization_lost"
    if "dock" in lowered or "transition" in lowered or "latch" in lowered:
        return "docking_failure"
    if "cancel" in lowered:
        return "cancelled"
    return "controller_failure"


def configuration_hash(paths: list[Path], parameters: dict) -> str:
    digest = hashlib.sha256()
    for path in sorted(paths, key=lambda value: str(value)):
        digest.update(str(path).encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    digest.update(json.dumps(parameters, sort_keys=True, separators=(",", ":")).encode())
    return digest.hexdigest()


def _git_revision(root: Path, allow_dirty: bool) -> str:
    revision = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain"], cwd=root, text=True).strip()
    if dirty and not allow_dirty:
        raise RuntimeError("refusing experiment run from a dirty worktree; use --allow-dirty for engineering runs")
    return revision + ("-dirty" if dirty else "")


def _stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
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


def run_batch(
    root: Path,
    design_path: Path,
    raw_root: Path,
    artifact_root: Path,
    catalog_path: Path,
    max_trials: int | None,
    wall_watchdog_s: float,
    allow_dirty: bool,
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
        layout_index = int(spec.layout_id.rsplit("-", 1)[1])
        scenario = make_confirmatory_scenario(
            spec.family, layout_index, spec.world_seed)
        trial_artifacts = artifact_root / spec.trial_id
        trial_artifacts.mkdir(parents=True, exist_ok=True)
        world_path, scenario_manifest = export_confirmatory_sdf(
            spec.family, layout_index, spec.world_seed,
            trial_artifacts / "world.sdf", catalog_path)
        map_yaml, map_image = export_occupancy_map(
            scenario.grid, trial_artifacts / "map.yaml")
        start_x, start_y = scenario.grid.cell_center(*scenario.start)
        parameters = {
            "deadline_s": 300.0,
            "wall_watchdog_s": wall_watchdog_s,
            "method": spec.method,
            "energy_measure": "10_newton_per_pod_absolute_speed_integral_proxy",
            "sequential_execution": True,
        }
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
        launch_log = (trial_artifacts / "launch.log").open("w", encoding="utf-8")
        process = subprocess.Popen(
            ["ros2", "launch", "modular_robot_bringup", "demo.launch.py",
             f"world:={world_path.resolve()}", f"map:={map_yaml.resolve()}",
             f"initial_x:={start_x}", f"initial_y:={start_y}"],
            cwd=root, stdout=launch_log, stderr=subprocess.STDOUT,
            text=True, start_new_session=True,
        )
        try:
            goal = scenario.grid.cell_center(*scenario.goal)
            observation = execute_mission(goal, spec.method, 300.0, wall_watchdog_s)
            if process.poll() is not None and not observation.completed:
                observation.terminal_status = "process_crash"
                observation.message += f"; launch exited {process.returncode}"
        except Exception as exc:
            observation = MissionObservation(
                "infrastructure_failure", False, f"runner exception: {exc}",
                0.0, 0.0)
        finally:
            _stop_process(process)
            launch_log.close()
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
            covariance_trace=observation.covariance_trace,
            odometry_history=observation.odometry_history,
            command_history=observation.command_history,
            topology_history=observation.topology_history,
            execution_state_history=observation.execution_state_history,
            final_execution_state=observation.final_execution_state,
            unrecovered_fault=observation.unrecovered_fault,
            topology_revision=observation.topology_revision,
            sensing_revision=observation.sensing_revision,
            map_revision=observation.map_revision,
            planned_route_signature=observation.planned_route_signature,
            planned_transition_sites=observation.planned_transition_sites,
            notes=[observation.message, f"scenario_manifest={scenario_manifest}"],
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
    parser.add_argument("--allow-dirty", action="store_true")
    return parser


def main() -> None:
    args = _parser().parse_args()
    result = run_batch(
        args.root.resolve(), args.design.resolve(), args.raw.resolve(),
        args.artifacts.resolve(), args.catalog.resolve(), args.max_trials,
        args.wall_watchdog, args.allow_dirty)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
