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


def _wrap(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def site_alignment_command(
    current: PlanarPose, site: PlanarPose, position_tolerance: float,
    yaw_tolerance: float, translating: bool,
    max_linear: float = 0.15, max_angular: float = 0.6,
) -> tuple[float, float, bool, bool]:
    """Differential command bringing the assembly onto a planned transition site.

    Transition feasibility is checked at the exact planned site pose, but the
    path follower arrives anywhere within its goal tolerance and ignores
    heading, so the transformation could run at a pose nobody checked. Returns
    ``(linear, angular, translating, aligned)``: close position first (forward
    or reverse, whichever faces the site), then turn in place to the planned
    yaw. ``translating`` carries hysteresis so a small drift while turning
    does not restart translation until it doubles the tolerance.
    """
    dx, dy = site.x - current.x, site.y - current.y
    distance = hypot(dx, dy)
    if distance > (position_tolerance if translating else 2.0 * position_tolerance):
        bearing = _wrap(atan2(dy, dx) - current.yaw)
        direction = 1.0
        if abs(bearing) > 1.5707963267948966:
            bearing, direction = _wrap(bearing - 3.141592653589793), -1.0
        angular = max(-max_angular, min(max_angular, 1.5 * bearing))
        if abs(bearing) > 0.15:
            return 0.0, angular, True, False
        return direction * min(max_linear, max(0.03, 0.8 * distance)), angular, True, False
    yaw_error = _wrap(site.yaw - current.yaw)
    if abs(yaw_error) <= yaw_tolerance:
        return 0.0, 0.0, False, True
    magnitude = min(max_angular, max(0.15, 1.5 * abs(yaw_error)))
    return 0.0, magnitude if yaw_error > 0 else -magnitude, False, False


def goal_position_reached(current: PlanarPose, goal: PlanarPose,
                          tolerance: float) -> bool:
    """Independent mission completion check in the common map frame."""
    return tolerance > 0 and hypot(current.x - goal.x, current.y - goal.y) <= tolerance
