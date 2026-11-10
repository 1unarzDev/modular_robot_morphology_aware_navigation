"""Batch runner for ExperimentChoice doorway designs (EXPLORATORY).

Reached through `mission_batch.main` when the frozen design's kind is
`expchoice_doorway_v0`; every other design keeps `mission_batch.run_batch`.
The trial lifecycle is the same one, built from the same helpers: launch the
stack, run the mission in its own process, read process deaths and transport
failures before teardown, invalidate on a clock jump, infer contact and
localization loss from evaluator truth, and retry only pre-mission
infrastructure failures. What differs is the scenario (an exact doorway
course), the stack file (a frozen Nav2 parameter file chosen per trial) and
the driver (stock Nav2's NavigateToPose).

NOT YET RUN in a simulator; `prepare_trial` and everything it calls is covered
by host tests, the launch loop is not.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import time
from typing import Any, Sequence

from .design import EXPCHOICE_DOORWAY_DESIGN_KIND, StudyDesign
from .doorway_design import RUNNABLE_MORPHOLOGIES
from .doorway_outcome import GOAL_TOLERANCE_M, doorway_outcome, goal_truth, motion_summary
from .doorway_scenarios import (
    MAP_RESOLUTION_M, course_from_spec, export_doorway_map, obstacle_boxes,
)
from .evaluator_metrics import (
    clearance_metrics, localization_error_track, localization_errors,
    localization_loss_episodes, module_pose_samples,
)
from .records import TrialManifest, TrialRecord, TrialStore
from .sdf_export import export_doorway_sdf

STACK_PARAMETERS = ("deadline_s", "wall_watchdog_s", "energy_measure",
                    "sequential_execution", "max_infrastructure_retries",
                    "navigator", "nav2_config", "nav2_config_sha256", "morphology")


def prepare_trial(root: Path, spec, trial_artifacts: Path, catalog_path: Path,
                  wall_watchdog_s: float, max_infrastructure_retries: int) -> dict[str, Any]:
    """Everything a doorway trial needs before a process is started.

    Writes the world, its manifest and the map; checks the Nav2 parameter file
    against the hash the design froze; returns the launch arguments, the
    record's parameters and the evaluator's obstacle boxes.
    """
    from .mission_batch import configuration_hash

    if spec.morphology not in RUNNABLE_MORPHOLOGIES:
        raise ValueError(
            f"trial {spec.trial_id}: no world spawns {spec.morphology} assembled")
    course = course_from_spec(spec)
    nav2_params = (root / spec.nav2_config).resolve()
    digest = hashlib.sha256(nav2_params.read_bytes()).hexdigest()
    if digest != spec.nav2_config_sha256:
        raise ValueError(
            f"trial {spec.trial_id}: {spec.nav2_config} has sha256 {digest}, "
            f"the design froze {spec.nav2_config_sha256}")
    trial_artifacts.mkdir(parents=True, exist_ok=True)
    world_path, scenario_manifest = export_doorway_sdf(
        course, trial_artifacts / "world.sdf", catalog_path, spec.friction_seed)
    map_yaml, map_image = export_doorway_map(course, trial_artifacts / "map.yaml")
    disturbance = json.loads(
        scenario_manifest.read_text(encoding="utf-8"))["physical_disturbance"]
    start_x, start_y, start_yaw = course.start
    start = (start_x + disturbance["initial_dx_m"],
             start_y + disturbance["initial_dy_m"],
             start_yaw + disturbance["initial_yaw_rad"])
    parameters = {
        "deadline_s": float(spec.deadline_s),
        "wall_watchdog_s": wall_watchdog_s,
        "method": spec.method,
        "arm": spec.arm,
        "morphology": spec.morphology,
        "navigator": "nav2_navigate_to_pose_default_behavior_tree",
        "nav2_config": spec.nav2_config,
        "nav2_config_sha256": digest,
        "energy_measure": "10_newton_per_pod_absolute_speed_integral_proxy",
        "sequential_execution": True,
        "world_seed": spec.world_seed,
        "sensing_seed": spec.sensing_seed,
        "friction_seed": spec.friction_seed,
        "fault_seed": spec.fault_seed,
        "physical_disturbance": disturbance,
        "max_infrastructure_retries": max_infrastructure_retries,
        "run_order": spec.run_order,
        "course_id": course.course_id,
        "course": course.params(),
        "map_resolution_m": MAP_RESOLUTION_M,
        "map_exactness": course.map_exactness(),
        "scoring_goal_tolerance_m": GOAL_TOLERANCE_M,
    }
    stack_paths = [catalog_path, nav2_params,
                   root / "src/modular_robot_bringup/config/localization.yaml"]
    parameters["stack_configuration_hash"] = configuration_hash(
        stack_paths, {key: parameters[key] for key in STACK_PARAMETERS})
    config_paths = [*stack_paths, world_path, scenario_manifest, map_yaml, map_image]
    launch_arguments = [
        f"world:={world_path.resolve()}", f"map:={map_yaml.resolve()}",
        # AMCL's prior; the robot spawns at the same pose.
        f"initial_x:={start[0]}", f"initial_y:={start[1]}", f"initial_yaw:={start[2]}",
        f"transition_environment:={scenario_manifest.resolve()}",
        "station_gated_visibility:=false",
        f"nav2_params:={nav2_params}",
    ]
    return {
        "course": course, "parameters": parameters,
        "configuration_hash": configuration_hash(config_paths, parameters),
        "launch_arguments": launch_arguments,
        "obstacle_boxes": obstacle_boxes(course),
        "scenario_manifest": scenario_manifest,
    }


def run_doorway_batch(
    root: Path, design: StudyDesign, raw_root: Path, artifact_root: Path,
    catalog_path: Path, max_trials: int | None, wall_watchdog_s: float,
    allow_dirty: bool, max_infrastructure_retries: int = 2,
    only_trial_ids: Sequence[str] | None = None,
) -> dict:
    from . import mission_batch as shared
    from .doorway_trial import execute_doorway_mission_isolated
    from .mission_trial import MissionObservation

    if design.design_kind != EXPCHOICE_DOORWAY_DESIGN_KIND:
        raise ValueError(f"not a doorway design: {design.design_kind}")
    store = TrialStore(raw_root)
    pending = shared.pending_trials(design, store, only_trial_ids)
    if max_trials is not None:
        pending = pending[:max_trials]
    revision = shared._git_revision(root, allow_dirty)
    written = []
    for spec in pending:
        setup_start = time.monotonic()
        trial_artifacts = artifact_root / spec.trial_id
        prepared = prepare_trial(root, spec, trial_artifacts, catalog_path,
                                 wall_watchdog_s, max_infrastructure_retries)
        course, parameters = prepared["course"], prepared["parameters"]
        manifest = TrialManifest(
            revision, prepared["configuration_hash"], design.design_hash,
            {
                "python": platform.python_version(),
                "platform": platform.platform(),
                "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
                "container_image": os.environ.get("MORPHOLOGY_CONTAINER_IMAGE", "unknown"),
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
                 *prepared["launch_arguments"]],
                cwd=root, stdout=launch_log, stderr=subprocess.STDOUT,
                text=True, start_new_session=True,
            )
            try:
                observation = execute_doorway_mission_isolated(
                    course.goal, 0.0, parameters["deadline_s"], wall_watchdog_s)
                launch_returncode = process.poll()
                if launch_returncode is not None and not observation.completed:
                    observation.terminal_status = "process_crash"
                    observation.message += f"; launch exited {launch_returncode}"
            except Exception as exc:
                launch_returncode = process.poll()
                observation = MissionObservation(
                    "infrastructure_failure", False, f"runner exception: {exc}", 0.0, 0.0)
            finally:
                premature_deaths = shared.stack_process_deaths(launch_log_path)
                teardown_start = time.monotonic()
                shared._stop_process(process)
                launch_log.close()
                cleared_segments = shared.clear_unused_dds_segments()
                teardown_wall_s = time.monotonic() - teardown_start
            failed_ports = shared.transport_initialization_failures(launch_log_path)
            if failed_ports and not observation.completed:
                observation.terminal_status = "infrastructure_failure"
                observation.message += (
                    "; DDS transport initialization failed on "
                    + ", ".join(sorted(failed_ports)))
            if (premature_deaths and not observation.completed
                    and observation.terminal_status != "infrastructure_failure"):
                observation.terminal_status = "infrastructure_failure"
                observation.message += "; stack process died: " + ", ".join(
                    death["process"] for death in premature_deaths)
            clock_fault = shared.mission_clock_discontinuity(observation.controller_diagnostics)
            if clock_fault and observation.terminal_status != "infrastructure_failure":
                observation.terminal_status = "infrastructure_failure"
                observation.completed = False
                observation.message += f"; {clock_fault}"
            diagnostics = observation.controller_diagnostics
            truth_samples = diagnostics.get("pod_drive_diagnostic_samples", [])
            clearance = clearance_metrics(
                module_pose_samples(diagnostics), prepared["obstacle_boxes"])
            diagnostics["evaluator_clearance"] = clearance
            loss_episodes = localization_loss_episodes(localization_error_track(
                observation.localization_history, truth_samples))
            diagnostics["evaluator_localization_loss"] = loss_episodes
            diagnostics["expchoice"] = {
                "goal_truth": goal_truth(truth_samples, course.goal),
                "motion": motion_summary(
                    observation.command_history, observation.odometry_history),
            }
            status, completed, note = doorway_outcome(
                observation.terminal_status, observation.completed,
                clearance["contact_events"], loss_episodes)
            if note:
                observation.terminal_status, observation.completed = status, completed
                observation.message += f"; {note}"
            attempt_record = {
                "attempt": attempt_index + 1,
                "terminal_status": observation.terminal_status,
                "message": observation.message,
                "simulated_duration_s": observation.simulated_duration_s,
                "wall_duration_s": observation.wall_duration_s,
                "phase_timing": {**observation.phase_timing,
                                 "teardown_wall_s": teardown_wall_s},
                "launch_log": str(launch_log_path),
                "transport_failed_ports": sorted(failed_ports),
                "stale_dds_segments_cleared": len(cleared_segments),
                "stack_process_deaths": premature_deaths,
            }
            infrastructure_attempts.append(attempt_record)
            with attempt_ledger_path.open("a", encoding="utf-8") as ledger:
                ledger.write(json.dumps(attempt_record, sort_keys=True) + "\n")
                ledger.flush()
                os.fsync(ledger.fileno())
            if (attempt_index >= max_infrastructure_retries
                    or not shared.retryable_infrastructure_failure(
                        observation.terminal_status, launch_returncode)):
                break
            time.sleep(2.0)
        record = TrialRecord(
            schema_version=1, spec=spec, manifest=manifest,
            terminal_status=observation.terminal_status,
            completed=observation.completed,
            simulated_duration_s=observation.simulated_duration_s,
            wall_duration_s=observation.wall_duration_s,
            deadline_s=float(spec.deadline_s),
            mechanical_work_j=observation.mechanical_work_j,
            covariance_trace=observation.covariance_trace,
            odometry_history=observation.odometry_history,
            command_history=observation.command_history,
            localization_history=observation.localization_history,
            minimum_clearance_m=clearance["minimum_clearance_m"],
            collision_count=clearance["collision_count"],
            localization_error_m=localization_errors(
                observation.localization_history, truth_samples),
            pod_alignment_history=observation.pod_alignment_history,
            recovery_actions=observation.recovery_actions,
            infrastructure_attempts=infrastructure_attempts,
            controller_diagnostics=diagnostics,
            topology_history=observation.topology_history,
            execution_state_history=observation.execution_state_history,
            final_execution_state=observation.final_execution_state,
            final_morphology=observation.final_morphology,
            unrecovered_fault=observation.unrecovered_fault,
            phase_timing={**observation.phase_timing, "setup_wall_s": setup_wall_s,
                          "teardown_wall_s": teardown_wall_s},
            topology_revision=observation.topology_revision,
            sensing_revision=observation.sensing_revision,
            notes=[observation.message,
                   f"scenario_manifest={prepared['scenario_manifest']}",
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
