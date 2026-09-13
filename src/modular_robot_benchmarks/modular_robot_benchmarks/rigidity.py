from __future__ import annotations

from bisect import bisect_left
from math import acos, atan2, cos, sin, sqrt


POSE_FIELDS = (
    "world_x", "world_y", "world_z", "world_roll", "world_pitch", "world_yaw",
)


def _rotation(sample: dict) -> tuple[tuple[float, float, float], ...]:
    """Return the ZYX rotation matrix represented by a Gazebo pose sample."""
    roll, pitch, yaw = (float(sample[name]) for name in POSE_FIELDS[3:])
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw), sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _transpose(matrix):
    return tuple(zip(*matrix))


def _multiply(left, right):
    return tuple(tuple(sum(left[i][k] * right[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


def _matvec(matrix, vector):
    return tuple(sum(matrix[i][k] * vector[k] for k in range(3)) for i in range(3))


def _relative(reference: dict, pod: dict):
    reference_rotation = _rotation(reference)
    relative_rotation = _multiply(_transpose(reference_rotation), _rotation(pod))
    displacement = tuple(float(pod[name]) - float(reference[name])
                         for name in POSE_FIELDS[:3])
    return _matvec(_transpose(reference_rotation), displacement), relative_rotation


def _rotation_distance(left, right) -> float:
    delta = _multiply(_transpose(left), right)
    return acos(max(-1.0, min(1.0, (sum(delta[i][i] for i in range(3)) - 1.0) / 2.0)))


def _interpolated_pose(samples: list[dict], time_s: float,
                       tolerance_s: float) -> dict | None:
    """Interpolate a reference pose; per-pod evaluator streams are asynchronous."""
    times = [float(sample["time_s"]) for sample in samples]
    index = bisect_left(times, time_s)
    if index == 0 or index == len(samples):
        nearest = samples[0] if index == 0 else samples[-1]
        return nearest if abs(float(nearest["time_s"]) - time_s) <= tolerance_s else None
    before, after = samples[index - 1], samples[index]
    before_time, after_time = float(before["time_s"]), float(after["time_s"])
    if after_time - before_time > 2.0 * tolerance_s:
        return None
    fraction = (time_s - before_time) / (after_time - before_time)
    result = {"time_s": time_s}
    for name in POSE_FIELDS[:3]:
        result[name] = float(before[name]) + fraction * (
            float(after[name]) - float(before[name]))
    for name in POSE_FIELDS[3:]:
        start = float(before[name])
        delta = atan2(sin(float(after[name]) - start), cos(float(after[name]) - start))
        result[name] = start + fraction * delta
    return result


def qualify_pod_rigidity(
    samples: list[dict], start_time_s: float, end_time_s: float,
    pods: tuple[str, ...] = tuple(f"pod_{index}" for index in range(6)),
    reference_pod: str = "pod_0", synchronization_tolerance_s: float = 0.30,
    minimum_samples_per_pod: int = 3, maximum_translation_drift_m: float = 0.015,
    maximum_rotation_drift_rad: float = 0.03,
) -> dict:
    """Evaluate post-latch relative-transform stability from evaluator-only poses."""
    selected = [sample for sample in samples
                if start_time_s <= float(sample.get("time_s", -1.0)) <= end_time_s
                and sample.get("pod") in pods
                and all(name in sample for name in POSE_FIELDS)]
    by_pod = {pod: sorted((sample for sample in selected if sample["pod"] == pod),
                          key=lambda value: value["time_s"]) for pod in pods}
    references = by_pod.get(reference_pod, [])
    metrics = {}
    for pod in pods:
        if pod == reference_pod:
            continue
        synchronized = []
        for sample in by_pod[pod]:
            reference = _interpolated_pose(
                references, float(sample["time_s"]), synchronization_tolerance_s)
            if reference is not None:
                synchronized.append(_relative(reference, sample))
        if synchronized:
            baseline_position, baseline_rotation = synchronized[0]
            translation = [sqrt(sum((position[i] - baseline_position[i]) ** 2
                                    for i in range(3)))
                           for position, _ in synchronized]
            rotation = [_rotation_distance(baseline_rotation, orientation)
                        for _, orientation in synchronized]
            maximum_translation = max(translation)
            maximum_rotation = max(rotation)
        else:
            maximum_translation = maximum_rotation = None
        enough = len(synchronized) >= minimum_samples_per_pod
        metrics[pod] = {
            "synchronized_samples": len(synchronized),
            "maximum_translation_drift_m": maximum_translation,
            "maximum_rotation_drift_rad": maximum_rotation,
            "passed": bool(enough and maximum_translation <= maximum_translation_drift_m
                           and maximum_rotation <= maximum_rotation_drift_rad),
        }
    return {
        "passed": bool(metrics and all(value["passed"] for value in metrics.values())),
        "source": "evaluator_only_gazebo_pose",
        "reference_pod": reference_pod,
        "window": {"start_time_s": start_time_s, "end_time_s": end_time_s},
        "thresholds": {
            "synchronization_tolerance_s": synchronization_tolerance_s,
            "minimum_samples_per_pod": minimum_samples_per_pod,
            "maximum_translation_drift_m": maximum_translation_drift_m,
            "maximum_rotation_drift_rad": maximum_rotation_drift_rad,
        },
        "pods": metrics,
    }
