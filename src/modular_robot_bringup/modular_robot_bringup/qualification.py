from dataclasses import dataclass
from math import atan2, cos, hypot, sin


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    yaw: float


def motion_delta(start: PlanarPose, end: PlanarPose) -> dict[str, float]:
    dx, dy = end.x - start.x, end.y - start.y
    return {
        "forward_m": cos(start.yaw) * dx + sin(start.yaw) * dy,
        "lateral_m": -sin(start.yaw) * dx + cos(start.yaw) * dy,
        "translation_m": hypot(dx, dy),
        "yaw_rad": atan2(sin(end.yaw - start.yaw), cos(end.yaw - start.yaw)),
    }


def qualification_pass(stages: dict[str, dict[str, float]]) -> bool:
    positive = stages["positive_yaw"]
    negative = stages["negative_yaw"]
    forward = stages["forward"]
    reverse = stages["reverse"]
    return (
        positive["yaw_rad"] >= 0.10
        and negative["yaw_rad"] <= -0.10
        and positive["translation_m"] <= 0.12
        and negative["translation_m"] <= 0.12
        and positive["translation_m"] / abs(positive["yaw_rad"]) <= 0.25
        and negative["translation_m"] / abs(negative["yaw_rad"]) <= 0.25
        and forward["forward_m"] >= 0.08
        and reverse["forward_m"] <= -0.08
        and abs(forward["lateral_m"]) <= 0.08
        and abs(reverse["lateral_m"]) <= 0.08
        and abs(forward["yaw_rad"]) <= 0.15
        and abs(reverse["yaw_rad"]) <= 0.15
    )


def goal_position_reached(current: PlanarPose, goal: PlanarPose,
                          tolerance: float) -> bool:
    """Independent mission completion check in the common map frame."""
    return tolerance > 0 and hypot(current.x - goal.x, current.y - goal.y) <= tolerance
