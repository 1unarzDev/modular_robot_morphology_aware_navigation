from collections.abc import Callable
from dataclasses import dataclass
from math import atan2, cos, hypot, sin


# Assembled motion commanded at a reconfiguration site once a morphology is
# committed, as (name, linear_mps, angular_rps, duration_s). This is the
# executed copy of `post_transition_verification` in the morphology catalog,
# which the planner sweeps when deciding whether a site can host a
# transformation. `tests/test_post_transition_qualification.py` binds the two,
# so a site the planner accepted is a site this maneuver was checked against.
VERIFICATION_SEQUENCE = (
    ("positive_yaw", 0.0, 0.25, 1.5),
    ("negative_yaw", 0.0, -0.25, 1.5),
    ("forward", 0.12, 0.0, 1.0),
    ("reverse", -0.12, 0.0, 1.0),
)


class CommandWindow:
    """Bounds one commanded maneuver stage in simulated time.

    The robot moves in simulated time, so bounding a stage with the wall clock
    delivers only ``duration_s * real_time_factor`` simulated seconds of
    motion. Travel then scales with host load rather than with mechanics, and
    a healthy robot fails the gate whenever the simulator falls behind.

    The wall clock is retained only as a stall backstop: if the simulated clock
    stops advancing, the stage ends and is reported as a stall instead of
    blocking until the trial watchdog fires.
    """

    def __init__(self, duration_s: float, simulated_now_s: Callable[[], float],
                 wall_now_s: Callable[[], float], stall_grace_s: float) -> None:
        self._duration_s = float(duration_s)
        self._simulated_now_s = simulated_now_s
        self._wall_now_s = wall_now_s
        self._simulated_start_s = float(simulated_now_s())
        self._wall_deadline_s = (
            float(wall_now_s()) + self._duration_s + float(stall_grace_s))
        self.elapsed_s = 0.0

    def keep_commanding(self) -> bool:
        self.elapsed_s = float(self._simulated_now_s()) - self._simulated_start_s
        return self.elapsed_s < self._duration_s

    @property
    def stalled(self) -> bool:
        return (self.elapsed_s < self._duration_s
                and float(self._wall_now_s()) >= self._wall_deadline_s)


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
