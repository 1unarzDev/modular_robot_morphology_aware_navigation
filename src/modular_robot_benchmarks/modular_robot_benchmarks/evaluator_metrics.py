"""Evaluator-only outcome metrics computed after a trial ends.

Inputs combine autonomy estimates recorded by the observer with Gazebo truth
from evaluator-only diagnostics. Nothing here is reachable from localization,
planning, docking, or recovery.
"""
from __future__ import annotations

from bisect import bisect_left
from math import cos, hypot, pi, sin, sqrt
from typing import Any, Iterable, Mapping, Sequence

# Localization is logged at 2 Hz while each pod diagnostic carries a same-tick
# core pose, so truth is normally available within one diagnostic period.
LOCALIZATION_SYNC_TOLERANCE_S = 0.25


def core_truth_track(samples: Iterable[Mapping[str, Any]]) -> list[tuple[float, float, float]]:
    """Time-sorted, de-duplicated (time_s, x, y) core poses from pod diagnostics."""
    track: dict[float, tuple[float, float]] = {}
    for sample in samples:
        if not {"time_s", "core_world_x", "core_world_y"} <= sample.keys():
            continue
        track[float(sample["time_s"])] = (
            float(sample["core_world_x"]), float(sample["core_world_y"]))
    return [(time_s, *track[time_s]) for time_s in sorted(track)]


def localization_errors(
    localization_history: Iterable[Mapping[str, Any]],
    diagnostic_samples: Iterable[Mapping[str, Any]],
    tolerance_s: float = LOCALIZATION_SYNC_TOLERANCE_S,
) -> list[float]:
    """Planar map-frame estimate error against the nearest-in-time core truth.

    Generated worlds place walls, the occupancy map, the spawn pose, and the
    AMCL initial pose in one coordinate frame, so map and Gazebo world coincide.
    Estimates without truth inside the tolerance are omitted, not imputed.
    """
    track = core_truth_track(diagnostic_samples)
    times = [entry[0] for entry in track]
    errors = []
    for estimate in localization_history:
        time_s = float(estimate["time_s"])
        index = bisect_left(times, time_s)
        candidates = [track[i] for i in (index - 1, index) if 0 <= i < len(track)]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda entry: abs(entry[0] - time_s))
        if abs(nearest[0] - time_s) > tolerance_s:
            continue
        errors.append(hypot(float(estimate["x"]) - nearest[1],
                            float(estimate["y"]) - nearest[2]))
    return errors


# ---------------------------------------------------------------------------
# Collision and 3D clearance
#
# Gazebo has no contact sensors in this platform, so contact is inferred after
# the trial from evaluator module poses and the generated world geometry. Module
# boxes use yaw only; pod roll/pitch under suspension is a few degrees and is
# ignored. The ground plane is not an obstacle.
# ---------------------------------------------------------------------------

# Sensor-core base_link collision box, centered on the link origin (SDF).
CORE_COLLISION_BOX = ((0.0, 0.0, 0.0), (0.26, 0.20, 0.12))
# Articulated pod collision bounds relative to the pod model origin (SDF): body
# box plus conservative boxes around each 0.055 m radius, 0.035 m wide wheel.
POD_COLLISION_BOXES = (
    ((0.0, 0.0, 0.07), (0.18, 0.10, 0.10)),
    ((0.0, 0.07, 0.055), (0.11, 0.035, 0.11)),
    ((0.0, -0.07, 0.055), (0.11, 0.035, 0.11)),
)
# Obstacles farther than this from a module center cannot set the minimum.
OBSTACLE_SEARCH_RADIUS_M = 1.0
# Clearance at or below zero is contact; consecutive contact samples of one
# module form one collision event.
CONTACT_CLEARANCE_M = 0.0
COLLISION_EVENT_GAP_S = 0.5


def _yaw_box_corners(center, size, yaw) -> list[tuple[float, float]]:
    cx, cy = center[0], center[1]
    hx, hy = size[0] / 2.0, size[1] / 2.0
    c, s = cos(yaw), sin(yaw)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]


def _segment_distance(p, q, a, b) -> float:
    def point_segment(point, start, end):
        vx, vy = end[0] - start[0], end[1] - start[1]
        length_sq = vx * vx + vy * vy
        t = 0.0 if length_sq == 0.0 else max(0.0, min(1.0, (
            (point[0] - start[0]) * vx + (point[1] - start[1]) * vy) / length_sq))
        return hypot(point[0] - start[0] - t * vx, point[1] - start[1] - t * vy)
    return min(point_segment(p, a, b), point_segment(q, a, b),
               point_segment(a, p, q), point_segment(b, p, q))


def _polygons_overlap(first, second) -> bool:
    for polygon in (first, second):
        for index in range(len(polygon)):
            x0, y0 = polygon[index]
            x1, y1 = polygon[(index + 1) % len(polygon)]
            axis = (y0 - y1, x1 - x0)
            first_proj = [axis[0] * x + axis[1] * y for x, y in first]
            second_proj = [axis[0] * x + axis[1] * y for x, y in second]
            if max(first_proj) < min(second_proj) or max(second_proj) < min(first_proj):
                return False
    return True


def box_clearance(
    module_center: Sequence[float], module_size: Sequence[float], module_yaw: float,
    obstacle_center: Sequence[float], obstacle_size: Sequence[float],
) -> float:
    """Euclidean gap between a yawed module box and an axis-aligned obstacle box.

    Returns 0.0 when the boxes touch or interpenetrate.
    """
    module = _yaw_box_corners(module_center, module_size, module_yaw)
    obstacle = _yaw_box_corners(obstacle_center, obstacle_size, 0.0)
    if _polygons_overlap(module, obstacle):
        planar = 0.0
    else:
        planar = min(_segment_distance(module[i], module[(i + 1) % 4],
                                       obstacle[j], obstacle[(j + 1) % 4])
                     for i in range(4) for j in range(4))
    vertical = max(0.0,
                   abs(module_center[2] - obstacle_center[2])
                   - (module_size[2] + obstacle_size[2]) / 2.0)
    return sqrt(planar * planar + vertical * vertical)


def _module_boxes(sample: Mapping[str, Any], boxes) -> list[tuple[tuple, tuple, float]]:
    x, y, z, yaw = (float(sample[key]) for key in ("x", "y", "z", "yaw"))
    c, s = cos(yaw), sin(yaw)
    return [((x + c * ox - s * oy, y + s * ox + c * oy, z + oz), size, yaw)
            for (ox, oy, oz), size in boxes]


def module_pose_samples(diagnostics: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Evaluator module poses: pods from the pose ledger, core from diagnostics."""
    samples = []
    for sample in diagnostics.get("pod_pose_samples", []):
        samples.append({"time_s": float(sample["time_s"]), "module": str(sample["pod"]),
                        "x": sample["world_x"], "y": sample["world_y"],
                        "z": sample["world_z"], "yaw": sample["world_yaw"]})
    core_times = set()
    for sample in diagnostics.get("pod_drive_diagnostic_samples", []):
        if not all(f"core_world_{axis}" in sample for axis in ("x", "y", "z", "yaw")):
            continue
        time_s = float(sample["time_s"])
        if time_s in core_times:
            continue
        core_times.add(time_s)
        samples.append({"time_s": time_s, "module": "core",
                        "x": sample["core_world_x"], "y": sample["core_world_y"],
                        "z": sample["core_world_z"], "yaw": sample["core_world_yaw"]})
    return sorted(samples, key=lambda value: (value["time_s"], value["module"]))


def world_obstacle_boxes(scenario) -> list[tuple[tuple, tuple]]:
    """Static obstacle boxes exactly as `sdf_export` places them in the world."""
    from morphology_planner.grid import OCCUPIED

    grid = scenario.grid
    boxes = []
    for y in range(grid.height):
        for x in range(grid.width):
            if grid.value(x, y) == OCCUPIED:
                wx, wy = grid.cell_center(x, y)
                boxes.append(((wx, wy, 0.5), (grid.resolution, grid.resolution, 1.0)))
    boxes.extend((tuple(box.center), tuple(box.size)) for box in scenario.transition_obstacles)
    return boxes


def clearance_metrics(
    module_samples: Iterable[Mapping[str, Any]],
    obstacles: Sequence[tuple[Sequence[float], Sequence[float]]],
) -> dict[str, Any]:
    """Minimum 3D clearance and contact events over evaluator module poses."""
    minimum = None
    minimum_at = None
    # Each debounced contact is retained with the module and time that produced
    # it. The loop already distinguishes them in order to count events; keeping
    # them makes contact-phase attribution a property of the record instead of
    # something a consumer has to infer from the mission's execution outcome.
    contact_events: list[dict[str, Any]] = []
    last_contact: dict[str, float] = {}
    evaluated = 0
    for sample in module_samples:
        boxes = _module_boxes(
            sample, (CORE_COLLISION_BOX,) if sample["module"] == "core" else POD_COLLISION_BOXES)
        x, y = float(sample["x"]), float(sample["y"])
        nearby = [obstacle for obstacle in obstacles
                  if hypot(obstacle[0][0] - x, obstacle[0][1] - y) <= OBSTACLE_SEARCH_RADIUS_M]
        if not nearby:
            continue
        evaluated += 1
        clearance = min(box_clearance(center, size, yaw, obstacle[0], obstacle[1])
                        for center, size, yaw in boxes for obstacle in nearby)
        if minimum is None or clearance < minimum:
            minimum, minimum_at = clearance, {"time_s": sample["time_s"],
                                              "module": sample["module"]}
        if clearance <= CONTACT_CLEARANCE_M:
            previous = last_contact.get(sample["module"])
            if previous is None or sample["time_s"] - previous > COLLISION_EVENT_GAP_S:
                contact_events.append({"module": sample["module"],
                                       "time_s": float(sample["time_s"]),
                                       "clearance_m": float(clearance)})
            last_contact[sample["module"]] = sample["time_s"]
    contact_events.sort(key=lambda event: (event["time_s"], event["module"]))
    return {"minimum_clearance_m": minimum, "minimum_clearance_at": minimum_at,
            "collision_count": len(contact_events),
            "contact_events": contact_events, "evaluated_samples": evaluated}


def transition_calibration_pairs(
    attempts: Iterable[Mapping[str, Any]],
) -> tuple[list[float], list[int]]:
    """Predicted success probabilities and binary outcomes, one per attempt.

    The planner reports a pre-action failure probability; analysis scores the
    probability of success against success frequency.
    """
    predictions, outcomes = [], []
    for attempt in attempts:
        failure = float(attempt["predicted_failure_probability"])
        if not 0.0 <= failure <= 1.0:
            raise ValueError(f"predicted failure probability out of range: {failure}")
        predictions.append(1.0 - failure)
        outcomes.append(int(bool(attempt["success"])))
    return predictions, outcomes


# The observer samples localization at 2 Hz of simulated time.
LOCALIZATION_SAMPLE_RATE_HZ = 2.0
MINIMUM_LOCALIZATION_COVERAGE = 0.8
MINIMUM_TRUTH_MATCH_FRACTION = 0.9
TRUTH_SPAN_SLACK_S = 1.0
# Trials shorter than this never started navigation and carry no telemetry.
MINIMUM_AUDITED_DURATION_S = 5.0


# The acceptance gate shared by the planner predicate and `docking_acceptance`.
STATION_OBSERVABILITY_GATE = 0.015


def station_observability_audit(record, scenario, catalog,
                                heading_bins: int = 8) -> list[str]:
    """Reasons the station did not deliver the observability the planner predicted.

    ADR 0006: a transition site is chosen because the workspace fiducial station
    is predicted to see where the pods will dock. That prediction only means
    something if the station actually delivers it there, so this recomputes the
    prediction at the site the mission planned and checks it against the sensing
    the mission recorded.

    What this can and cannot catch. Prediction and the simulator's station share
    the geometry rule deliberately, so a bug in that rule would satisfy this
    audit; what it does catch is the site, the heading, the declared obstacles
    and the station pose disagreeing between planning and execution, and a pod
    the planner expected to be seen that the mission recorded as unobserved.

    A world that declares no station is exempt: it is not station-gated and
    keeps the behaviour it had before ADR 0006.
    """
    from morphology_planner import HybridState, station_sensing_provider

    if not scenario.fiducial_stations:
        return []
    transitions = {value.id: value for value in catalog.transitions}
    provider = station_sensing_provider(
        catalog, scenario.fiducial_stations, scenario.transition_obstacles,
        scenario.grid, heading_bins)
    problems = []
    for site in record.planned_transition_sites:
        transition = transitions.get(str(site.get("transition_id", "")))
        if transition is None:
            continue
        cell_x, cell_y = scenario.grid.world_to_cell(
            float(site["x"]), float(site["y"]))
        heading = round(
            float(site.get("yaw", 0.0)) % (2.0 * pi) * heading_bins
            / (2.0 * pi)) % heading_bins
        predicted = provider(
            transition, HybridState(cell_x, cell_y, heading, "compact_diff"))
        blind = sorted(pod for pod, state in predicted.items()
                       if not state.connector_visible)
        if blind:
            problems.append(
                f"{transition.id}: planner chose a site where the station "
                f"cannot see {', '.join(blind)}")
        problems.extend(_measured_disagreements(record, transition, predicted))
    return problems


def _measured_disagreements(record, transition, predicted) -> list[str]:
    """Pods the planner expected observed that the mission recorded otherwise."""
    snapshots = [value for value in record.pod_alignment_history
                 if value.get("execution_state") in ("TRANSITIONING", "READY")]
    if not snapshots:
        return []
    problems = []
    for pod in transition.moved_pods:
        if not predicted.get(pod) or not predicted[pod].connector_visible:
            continue
        observed = [
            snapshot["pods"][pod] for snapshot in snapshots
            if pod in snapshot.get("pods", {})
        ]
        if not observed:
            continue
        # The docking gate needs one accepted observation, not every sample.
        if any(bool(value.get("connector_visible"))
               and _trace(value) <= STATION_OBSERVABILITY_GATE
               for value in observed):
            continue
        best = min((_trace(value) for value in observed), default=float("inf"))
        problems.append(
            f"{transition.id}: station predicted to see {pod}, but no recorded "
            f"observation was both visible and within the gate "
            f"(best covariance trace {best:.4g})")
    return problems


def _trace(estimate: Mapping[str, Any]) -> float:
    return float(estimate.get("variance_x", 0.0)) + float(
        estimate.get("variance_y", 0.0)) + float(estimate.get("variance_yaw", 0.0))


def telemetry_audit(record) -> list[str]:
    """Reasons a terminal record's evaluator telemetry is incomplete or misaligned.

    An empty list means the localization and truth streams cover the simulated
    mission densely enough for outcome metrics. Records that ended before
    navigation began are exempt because they have no mission to measure.
    """
    duration = float(record.simulated_duration_s)
    if duration < MINIMUM_AUDITED_DURATION_S:
        return []
    diagnostics = record.controller_diagnostics
    problems = []
    dropped = int(diagnostics.get("pod_drive_diagnostic_samples_dropped", 0) or 0)
    if dropped:
        problems.append(f"evaluator diagnostics dropped {dropped} samples")
    track = core_truth_track(diagnostics.get("pod_drive_diagnostic_samples", []))
    if not track:
        problems.append("no evaluator core truth samples")
    estimates = list(record.localization_history)
    expected = duration * LOCALIZATION_SAMPLE_RATE_HZ
    if len(estimates) < MINIMUM_LOCALIZATION_COVERAGE * expected:
        problems.append(
            f"localization history has {len(estimates)} samples; expected about {expected:.0f}")
    if estimates and track:
        matched = len(localization_errors(estimates, diagnostics["pod_drive_diagnostic_samples"]))
        if matched < MINIMUM_TRUTH_MATCH_FRACTION * len(estimates):
            problems.append(
                f"only {matched}/{len(estimates)} localization samples have synchronized truth")
        first, last = float(estimates[0]["time_s"]), float(estimates[-1]["time_s"])
        if track[0][0] > first + TRUTH_SPAN_SLACK_S or track[-1][0] < last - TRUTH_SPAN_SLACK_S:
            problems.append(
                f"truth spans {track[0][0]:.1f}-{track[-1][0]:.1f} s but estimates span "
                f"{first:.1f}-{last:.1f} s")
    if len(record.localization_error_m) != len(localization_errors(
            estimates, diagnostics.get("pod_drive_diagnostic_samples", []))):
        problems.append("recorded localization errors do not match recomputation")
    return problems
