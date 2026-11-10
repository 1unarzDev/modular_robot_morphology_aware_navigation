"""Outcome rules for ExperimentChoice doorway trials (EXPLORATORY). No ROS imports.

A doorway trial is driven by stock Nav2's NavigateToPose, not by the hybrid
navigator, so its terminal status is read from the action's goal status and
error code instead of from a result message.
"""
from __future__ import annotations

from math import hypot
from typing import Any, Iterable, Mapping, Sequence

# action_msgs/GoalStatus
STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED = 4, 5, 6
# The manifest's goal radius for the rotation centre; scoring only. The
# controller's own goal checker is whatever the frozen parameter file says.
GOAL_TOLERANCE_M = 0.25
# Commands below this are the smoother's ramp tails, not a stretch of motion.
STRETCH_DEADBAND_MPS = 0.02


def nav2_terminal(goal_status: int, error_code: int, timed_out: bool) -> tuple[str, bool]:
    """(terminal_status, completed) for a NavigateToPose result.

    Nav2's planner error codes are 201-299 (nav2_msgs ComputePathToPose) and
    its controller codes 101-199 (FollowPath); only the range is used, so a
    renumbered code inside a range cannot change the label. An abort with no
    code, as when the behaviour tree runs out of recoveries, is a controller
    failure: the stack was asked to drive and gave up.
    """
    if timed_out:
        return "timeout", False
    if goal_status == STATUS_SUCCEEDED:
        return "completed", True
    if goal_status == STATUS_CANCELED:
        return "cancelled", False
    if 200 < int(error_code) < 300:
        return "planning_failure", False
    return "controller_failure", False


def doorway_outcome(status: str, completed: bool,
                    contact_events: Sequence[Mapping[str, Any]],
                    loss_episodes: Sequence[Mapping[str, Any]]) -> tuple[str, bool, str]:
    """`mission_batch.evaluator_outcome`, with planning failures eligible too.

    For the hybrid navigator a planning failure means the robot never moved, so
    the shared rule leaves it alone. Nav2 replans at 1 Hz while driving, so here
    a planning failure can follow motion and a contact before it is a collision.
    """
    from .mission_batch import evaluator_outcome

    if status != "planning_failure":
        return evaluator_outcome(status, completed, list(contact_events), list(loss_episodes))
    relabelled, done, note = evaluator_outcome(
        "controller_failure", completed, list(contact_events), list(loss_episodes))
    if not note:
        return status, completed, ""
    return relabelled, done, note.replace("controller_failure", "planning_failure")


def motion_summary(command_history: Iterable[Mapping[str, float]],
                   odometry_history: Iterable[Mapping[str, float]]) -> dict[str, Any]:
    """Stretches, reversals and reverse travel from the 2 Hz histories.

    A stretch is a maximal run of commanded longitudinal velocity of one sign
    (ExperimentChoice's definition); reversals = stretches - 1. Sampled at
    2 Hz, so a stroke shorter than half a second can be missed: a lower bound.
    """
    signs = []
    for sample in command_history:
        value = float(sample.get("body_linear_x", 0.0))
        if abs(value) < STRETCH_DEADBAND_MPS:
            continue
        sign = 1 if value > 0 else -1
        if not signs or signs[-1] != sign:
            signs.append(sign)
    forward = reverse = rotation = 0.0
    previous = None
    for sample in odometry_history:
        if previous is not None:
            step = hypot(float(sample["x"]) - float(previous["x"]),
                         float(sample["y"]) - float(previous["y"]))
            if float(sample.get("linear_x", 0.0)) < 0.0:
                reverse += step
            else:
                forward += step
            rotation += abs(float(sample.get("angular_z", 0.0))) * (
                float(sample["time_s"]) - float(previous["time_s"]))
        previous = sample
    return {"stretches": len(signs), "reversals": max(0, len(signs) - 1),
            "reverse_stretches": signs.count(-1),
            "forward_travel_m": forward, "reverse_travel_m": reverse,
            "rotation_rad_approx": rotation, "sample_rate_hz": 2.0}


def goal_truth(diagnostic_samples: Iterable[Mapping[str, Any]],
               goal_xy: Sequence[float]) -> dict[str, Any]:
    """Where the core truly ended relative to the goal (evaluator truth only)."""
    last = None
    for sample in diagnostic_samples:
        if {"time_s", "core_world_x", "core_world_y"} <= sample.keys():
            if last is None or float(sample["time_s"]) >= float(last["time_s"]):
                last = sample
    if last is None:
        return {"available": False}
    distance = hypot(float(last["core_world_x"]) - goal_xy[0],
                     float(last["core_world_y"]) - goal_xy[1])
    return {"available": True, "time_s": float(last["time_s"]),
            "x": float(last["core_world_x"]), "y": float(last["core_world_y"]),
            "yaw": float(last.get("core_world_yaw", 0.0)),
            "goal_distance_m": distance,
            "within_goal_tolerance": distance <= GOAL_TOLERANCE_M,
            "goal_tolerance_m": GOAL_TOLERANCE_M}


def footprint_edges(points: Sequence[Sequence[float]]) -> list[float]:
    """Sorted edge lengths of a published costmap footprint, frame-independent.

    For compact_diff Nav2 should be checking 0.58 x 0.68 m: the catalog's
    `navigation_footprint` (0.56 x 0.66) grown by Nav2's 0.01 m footprint padding.
    """
    count = len(points)
    return sorted(round(hypot(points[(index + 1) % count][0] - points[index][0],
                              points[(index + 1) % count][1] - points[index][1]), 3)
                  for index in range(count)) if count >= 2 else []
