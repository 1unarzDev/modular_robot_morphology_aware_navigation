from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin


INJECTABLE_FAILURE_STAGES = frozenset({
    "detach", "relocation", "latch", "manager_commit", "stale_feedback", "cancellation",
})


def injected_failure(configured: str, stage: str, pod: str = "") -> bool:
    """Match a deterministic fault stage, optionally scoped as ``stage:pod``."""
    configured = configured.strip()
    if not configured:
        return False
    name, separator, selected_pod = configured.partition(":")
    if name not in INJECTABLE_FAILURE_STAGES:
        raise ValueError(f"unknown failure injection stage: {name}")
    return name == stage and (not separator or selected_pod == pod)


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class VelocityCommand:
    linear: float = 0.0
    angular: float = 0.0


def wrap_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def compose_pose(parent: Pose2, relative: list[float]) -> Pose2:
    x, y, yaw = (float(value) for value in relative)
    return Pose2(
        parent.x + cos(parent.yaw) * x - sin(parent.yaw) * y,
        parent.y + sin(parent.yaw) * x + cos(parent.yaw) * y,
        wrap_angle(parent.yaw + yaw),
    )


def docking_command(
    current: Pose2,
    target: Pose2,
    position_tolerance: float,
    yaw_tolerance: float,
    max_linear: float,
    max_angular: float,
) -> tuple[VelocityCommand, bool]:
    """Closed-loop unicycle command with a final docking-yaw phase."""
    dx, dy = target.x - current.x, target.y - current.y
    distance = hypot(dx, dy)
    if distance > position_tolerance:
        bearing_error = wrap_angle(atan2(dy, dx) - current.yaw)
        linear = 0.0
        if abs(bearing_error) < 0.9:
            linear = min(max_linear, 1.25 * distance) * max(0.15, cos(bearing_error))
        angular = max(-max_angular, min(max_angular, 2.8 * bearing_error))
        return VelocityCommand(linear, angular), False
    yaw_error = wrap_angle(target.yaw - current.yaw)
    if abs(yaw_error) > yaw_tolerance:
        angular = max(-max_angular, min(max_angular, 2.5 * yaw_error))
        return VelocityCommand(0.0, angular), False
    return VelocityCommand(), True
