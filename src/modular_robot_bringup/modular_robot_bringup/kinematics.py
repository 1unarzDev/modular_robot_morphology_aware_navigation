from dataclasses import dataclass
from math import cos, sin


@dataclass(frozen=True)
class BodyTwist:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class PodCommand:
    linear: float
    angular: float = 0.0


def distribute_twist(
    twist: BodyTwist, pod_poses: dict[str, list[float]], max_speed: float
) -> dict[str, PodCommand]:
    """Project a body twist onto each docked pod's rolling direction."""
    output: dict[str, PodCommand] = {}
    for pod, pose in pod_poses.items():
        x, y, yaw = (float(value) for value in pose)
        point_vx = twist.x - twist.yaw * y
        point_vy = twist.y + twist.yaw * x
        speed = cos(yaw) * point_vx + sin(yaw) * point_vy
        output[pod] = PodCommand(max(-max_speed, min(max_speed, speed)))
    return output
