from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import signal
import subprocess
import sys
import time
from typing import Any

from .confirmatory_scenarios import make_confirmatory_scenario
from .design import (
    FAULT_MATRIX_DESIGN_KIND, GATE1_OUTCOMES_DESIGN_KIND, OutcomeCase, StudyDesign,
    fault_case_for_run, layout_generator_for, outcome_case_for_run,
)
from .evaluator_metrics import (
    clearance_metrics, localization_error_track, localization_errors,
    localization_loss_episodes, module_pose_samples, site_placement_errors,
    world_obstacle_boxes,
)
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
    # A method that found no plan never moved for a reason of its own; checked
    # before the transition keywords, because route-first's NoPathError names
    # "feasible transitions" and was being recorded as a docking failure.
    if (lowered.startswith("fixed spatial route cannot be adapted")
            or lowered.startswith("no collision-free hybrid plan")
            or lowered.startswith("no morphology-agnostic spatial route")):
        return "planning_failure"
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


# Statuses that describe what the robot did, and so can be re-attributed by
# evaluator evidence. Infrastructure, crashes and planning failures never moved
# the robot for a reason of its own and keep their status.
ROBOT_OUTCOME_STATUSES = frozenset({
    "completed", "timeout", "controller_failure", "docking_failure",
    "unsafe_topology", "motion_qualification_failure", "localization_lost",
})


def evaluator_outcome(
    status: str, completed: bool,
    contact_events: list[dict[str, Any]], loss_episodes: list[dict[str, Any]],
) -> tuple[str, bool, str]:
    """Terminal status after evaluator-only contact and localization evidence.

    The earliest evaluator-observed cause names the outcome. Any contact is a
    collision and any loss episode is localization loss, whichever began first,
    with one exception: a completed mission counts only a loss episode still in
    progress at the end, because a recovered estimate did not invalidate the
    completion while a diverged one asserted it against the wrong pose.
    Returns (status, completed, note); the note is empty when nothing changed.
    """
    if status not in ROBOT_OUTCOME_STATUSES:
        return status, completed, ""
    first_contact = min((float(event["time_s"]) for event in contact_events),
                        default=None)
    counted = [episode for episode in loss_episodes
               if episode["ongoing"] or not completed]
    loss_onset = float(counted[0]["onset_s"]) if counted else None
    if first_contact is not None and (loss_onset is None or first_contact <= loss_onset):
        return ("collision", False, "evaluator detected obstacle contact"
                + ("" if completed else f" at {first_contact:.1f} s, before the {status}"))
    if loss_onset is not None:
        if status == "localization_lost":
            return status, completed, ""
        return ("localization_lost", False,
                f"evaluator: estimate lost from {loss_onset:.1f} s"
                + (" to the end" if completed else f", before the {status}"))
    return status, completed, ""


def failure_injection_for(design: StudyDesign, spec) -> str:
    """Declared executor fault for a trial; empty outside the fault matrix."""
    if design.design_kind != FAULT_MATRIX_DESIGN_KIND:
        return ""
    return fault_case_for_run(spec.replicate).injection


def outcome_case_for(design: StudyDesign, spec) -> OutcomeCase | None:
    """Declared Gate 1 outcome stimulus for a trial; None outside that campaign."""
    if design.design_kind != GATE1_OUTCOMES_DESIGN_KIND:
        return None
    return outcome_case_for_run(spec.replicate)


def unperceived_obstacles(case: OutcomeCase | None, start_x: float, start_y: float,
                          ) -> tuple[tuple[str, tuple, tuple], ...]:
    """The case's world-only box, ahead of the nominal start along heading 0.

    Every route starts in heading bin 0 (east), and in the neutral layouts the
    Gate 1 campaign uses the first leg runs straight east from the start.
    """
    if case is None or case.unperceived_obstacle_ahead_m is None:
        return ()
    size = case.unperceived_obstacle_size
    return (("gate1_unperceived_obstacle",
             (start_x + case.unperceived_obstacle_ahead_m, start_y, size[2] / 2.0),
             size),)


def configuration_hash(paths: list[Path], parameters: dict) -> str:
    digest = hashlib.sha256()
    for path in sorted(paths, key=lambda value: str(value)):
        digest.update(str(path).encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    digest.update(json.dumps(parameters, sort_keys=True, separators=(",", ":")).encode())
    return digest.hexdigest()


TRANSPORT_FAILURE_PATTERN = re.compile(
    "open_and_lock_file failed"
    "|Failed init_port (?P<port>fastrtps_[A-Za-z0-9_]+)"
    "|RTPS_TRANSPORT_SHM.*Error")


def transport_initialization_failures(launch_log_path: Path) -> set[str]:
    """Shared-memory ports a participant in this trial could not open.

    A participant that cannot open its port is deaf and mute for the whole
    trial, so every outcome downstream of it describes the transport and not
    the robot. In the 2026-09-30 round trip exactly one trial of twenty hit
    this -- nine Nav2 nodes all failing to open `fastrtps_port18186` -- and it
    recorded as `controller_failure`, a robot outcome, on a run where the robot
    reached its site inside tolerance and never moved a pod.

    The evidence is in the launch log the runner already captures, so this needs
    no new instrumentation.
    """
    path = Path(launch_log_path)
    if not path.is_file():
        return set()
    matched = list(TRANSPORT_FAILURE_PATTERN.finditer(
        path.read_text(encoding="utf-8", errors="replace")))
    ports = {match.group("port") for match in matched if match.group("port")}
    if matched and not ports:
        # The transport failed without naming its port; still not the robot.
        return {"unnamed_port"}
    return ports


PROCESS_DEATH_PATTERN = re.compile(
    r"\[ERROR\] \[(?P<process>[^\]]+)\]: process has died \[pid \d+, exit code (?P<code>-?\d+)")


def stack_process_deaths(launch_log_path: Path) -> list[dict[str, Any]]:
    """Stack processes the launch log reports dead, read before teardown starts.

    Teardown kills every process, so deaths are only informative when read
    before it: anything this finds then died while the trial was running, which
    is process health the record otherwise cannot show while the launch itself
    stays alive.
    """
    path = Path(launch_log_path)
    if not path.is_file():
        return []
    return [{"process": match.group("process"), "exit_code": int(match.group("code"))}
            for match in PROCESS_DEATH_PATTERN.finditer(
                path.read_text(encoding="utf-8", errors="replace"))]


def unused_dds_segments(directory: Path = Path("/dev/shm")) -> list[Path]:
    """Fast-DDS segments no live process holds open.

    A stale segment poisons the next stack that wants its port, which is what
    cost the 2026-09-30 round trip a trial. Deleting segments blindly is the
    opposite fault and has already happened here: a teardown removed
    `/dev/shm/fastrtps_*` underneath live participants, and Gazebo then
    published `/clock` while no participant could discover anything.

    Holding both of those open at once is why this reads `/proc` rather than
    matching process names: a segment nobody has open or mapped cannot, by
    construction, be underneath a live participant.

    Mappings count as well as descriptors. Fast-DDS maps a port segment and
    closes its descriptor, so a descriptor-only check cleared the runner's own
    live ports after every trial of the 2026-10-27 round trip, and each later
    stack failed to open them: 0, 44, 44, 88, 132 transport errors in r0--r4.
    """
    directory = Path(directory)
    segments = sorted(directory.glob("fastrtps_*"))
    if not segments:
        return []
    held = set()
    for descriptors in Path("/proc").glob("[0-9]*/fd"):
        try:
            entries = list(descriptors.iterdir())
        except OSError:
            continue
        for descriptor in entries:
            try:
                target = os.readlink(descriptor)
            except OSError:
                continue
            if "fastrtps_" in target:
                held.add(target)
    for maps in Path("/proc").glob("[0-9]*/maps"):
        try:
            lines = maps.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        for line in lines:
            fields = line.split(maxsplit=5)
            if len(fields) == 6 and "fastrtps_" in fields[5]:
                held.add(fields[5].removesuffix(" (deleted)"))
    return [segment for segment in segments if str(segment) not in held]


def clear_unused_dds_segments(directory: Path = Path("/dev/shm")) -> list[str]:
    """Remove the segments nobody holds open. Returns what was cleared."""
    cleared = []
    for segment in unused_dds_segments(directory):
        try:
            segment.unlink()
        except OSError:
            continue
        cleared.append(segment.name)
    return cleared


def mission_clock_discontinuity(diagnostics: dict[str, Any]) -> str:
    """Why the simulated clock invalidates this mission, or "" if it does not.

    A jump in /clock during the mission means ROS stopped hearing from the
    simulator while it kept stepping (gz-transport forgot its publishers), so
    the robot ran uncommanded and the deadline was spent with nobody driving.
    Whatever followed is the transport's outcome, not the robot's.
    """
    jumps = [jump for jump in diagnostics.get("clock_discontinuities", [])
             if jump.get("during_mission")]
    if not jumps:
        return ""
    worst = max(jumps, key=lambda jump: jump["jump_s"])
    return (f"simulated clock jumped {worst['jump_s']:.1f} s at "
            f"{worst['from_s']:.1f} s during the mission ({len(jumps)} jump(s)); "
            "the bridge lost the simulator's publishers")


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
    from .mission_trial import MissionObservation, execute_mission_isolated

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
        generator = layout_generator_for(design.design_kind)
        scenario = make_confirmatory_scenario(
            spec.family, layout_index, spec.world_seed, generator=generator)
        trial_artifacts = artifact_root / spec.trial_id
        trial_artifacts.mkdir(parents=True, exist_ok=True)
        outcome_case = outcome_case_for(design, spec)
        nominal_start_x, nominal_start_y = scenario.grid.cell_center(*scenario.start)
        stimulus_boxes = unperceived_obstacles(
            outcome_case, nominal_start_x, nominal_start_y)
        world_path, scenario_manifest = export_confirmatory_sdf(
            spec.family, layout_index, spec.world_seed,
            trial_artifacts / "world.sdf", catalog_path, spec.friction_seed,
            stimulus_boxes, generator)
        map_yaml, map_image = export_occupancy_map(
            scenario.grid, trial_artifacts / "map.yaml")
        disturbance = json.loads(
            scenario_manifest.read_text(encoding="utf-8"))["physical_disturbance"]
        start_x = nominal_start_x + disturbance["initial_dx_m"]
        start_y = nominal_start_y + disturbance["initial_dy_m"]
        start_yaw = disturbance["initial_yaw_rad"]
        amcl_offset = ((outcome_case.amcl_initial_offset if outcome_case else None)
                       or (0.0, 0.0, 0.0))
        parameters = {
            "deadline_s": outcome_case.deadline_s if outcome_case else 300.0,
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
        failure_injection = (failure_injection_for(design, spec)
                             or (outcome_case.failure_injection if outcome_case else ""))
        if outcome_case is not None:
            parameters["gate1_outcome_case"] = asdict(outcome_case)
            parameters["gate1_unperceived_obstacles"] = [
                {"name": name, "center": list(center), "size": list(size)}
                for name, center, size in stimulus_boxes]
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
                # A container cannot read its own image digest; the run script
                # supplies it from `docker inspect`, and the Gate 1 audit flags
                # records where it did not.
                "container_image": os.environ.get(
                    "MORPHOLOGY_CONTAINER_IMAGE", "unknown"),
            },
            parameters,
        )
        # The evaluator sees the whole world, stimulus included.
        obstacle_boxes = world_obstacle_boxes(scenario) + [
            (tuple(center), tuple(size)) for _, center, size in stimulus_boxes]
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
                 # AMCL's prior only; the robot spawns at the true start. A
                 # Gate 1 localization case offsets it (a kidnapped robot).
                 f"initial_x:={start_x + amcl_offset[0]}",
                 f"initial_y:={start_y + amcl_offset[1]}",
                 f"initial_yaw:={start_yaw + amcl_offset[2]}",
                 # Static 3D transition obstacles as a known prior map, like
                 # the occupancy map; no live simulator state is exposed.
                 f"transition_environment:={scenario_manifest.resolve()}",
                 # ADR 0006: a scenario that declares a workspace fiducial
                 # station establishes connector observability through it, so
                 # the onboard cameras stop asserting it. Scenarios without a
                 # station keep the pre-ADR-0006 behaviour.
                 "station_gated_visibility:="
                 f"{'true' if scenario.fiducial_stations else 'false'}",
                 *([f"failure_injection:={failure_injection}"] if failure_injection else [])],
                cwd=root, stdout=launch_log, stderr=subprocess.STDOUT,
                text=True, start_new_session=True,
            )
            try:
                goal = scenario.grid.cell_center(*scenario.goal)
                observation = execute_mission_isolated(
                    goal, spec.method, parameters["deadline_s"], wall_watchdog_s,
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
                # Read before teardown, which kills every process it reaches.
                premature_deaths = stack_process_deaths(launch_log_path)
                teardown_start = time.monotonic()
                _stop_process(process)
                launch_log.close()
                # Only segments no live process holds open, so this cannot
                # repeat the teardown that deleted them under live participants.
                cleared_segments = clear_unused_dds_segments()
                teardown_wall_s = time.monotonic() - teardown_start
            # A participant that could not open its port was deaf and mute for
            # the whole trial, so what it produced is not a robot outcome.
            failed_ports = transport_initialization_failures(launch_log_path)
            if failed_ports and not observation.completed:
                observation.terminal_status = "infrastructure_failure"
                observation.message += (
                    "; DDS transport initialization failed on "
                    + ", ".join(sorted(failed_ports)))
            clock_fault = mission_clock_discontinuity(observation.controller_diagnostics)
            if clock_fault and observation.terminal_status != "infrastructure_failure":
                observation.terminal_status = "infrastructure_failure"
                observation.completed = False
                observation.message += f"; {clock_fault}"
            # Evaluator-only contact inference from module poses against the
            # generated world, and localization loss against core truth; see
            # `evaluator_outcome` for how each re-attributes the status.
            clearance = clearance_metrics(
                module_pose_samples(observation.controller_diagnostics), obstacle_boxes)
            observation.controller_diagnostics["evaluator_clearance"] = clearance
            loss_episodes = localization_loss_episodes(localization_error_track(
                observation.localization_history,
                observation.controller_diagnostics.get("pod_drive_diagnostic_samples", [])))
            observation.controller_diagnostics["evaluator_localization_loss"] = loss_episodes
            # ADR 0010: the site tolerance is nominal; record what was achieved.
            observation.controller_diagnostics["evaluator_site_placement"] = (
                site_placement_errors(
                    observation.planned_transition_sites,
                    observation.execution_state_history,
                    observation.controller_diagnostics.get(
                        "pod_drive_diagnostic_samples", [])))
            status, completed, note = evaluator_outcome(
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
            minimum_clearance_m=clearance["minimum_clearance_m"],
            collision_count=clearance["collision_count"],
            localization_error_m=localization_errors(
                observation.localization_history,
                observation.controller_diagnostics.get("pod_drive_diagnostic_samples", [])),
            pod_alignment_history=observation.pod_alignment_history,
            motion_qualifications=observation.motion_qualifications,
            predicted_transition_probabilities=observation.predicted_transition_probabilities,
            observed_transition_outcomes=observation.observed_transition_outcomes,
            transition_edge_decisions=observation.transition_edge_decisions,
            recovery_actions=observation.recovery_actions,
            infrastructure_attempts=infrastructure_attempts,
            controller_diagnostics=observation.controller_diagnostics,
            topology_history=observation.topology_history,
            execution_state_history=observation.execution_state_history,
            final_execution_state=observation.final_execution_state,
            final_morphology=observation.final_morphology,
            unrecovered_fault=observation.unrecovered_fault,
            recovery_probe=observation.recovery_probe,
            fired_injections=observation.fired_injections,
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
