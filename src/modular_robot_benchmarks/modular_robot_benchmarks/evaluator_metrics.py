"""Evaluator-only outcome metrics computed after a trial ends.

Inputs combine autonomy estimates recorded by the observer with Gazebo truth
from evaluator-only diagnostics. Nothing here is reachable from localization,
planning, docking, or recovery.
"""
from __future__ import annotations

from bisect import bisect_left
from math import hypot
from typing import Any, Iterable, Mapping

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


# The observer samples localization at 2 Hz of simulated time.
LOCALIZATION_SAMPLE_RATE_HZ = 2.0
MINIMUM_LOCALIZATION_COVERAGE = 0.8
MINIMUM_TRUTH_MATCH_FRACTION = 0.9
TRUTH_SPAN_SLACK_S = 1.0
# Trials shorter than this never started navigation and carry no telemetry.
MINIMUM_AUDITED_DURATION_S = 5.0


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
