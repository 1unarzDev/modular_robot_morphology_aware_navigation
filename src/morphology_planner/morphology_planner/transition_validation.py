from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, sin
from typing import Mapping


@dataclass(frozen=True)
class Pose3:
    x: float
    y: float
    z: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Box3:
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]


@dataclass(frozen=True)
class TrajectoryPoint:
    time_s: float
    pose: Pose3
    connector_visible: bool = True
    covariance_trace: float = 0.0
    latch_confirmed: bool = False


@dataclass(frozen=True)
class CollisionPart:
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]


@dataclass(frozen=True)
class ModuleTrajectory:
    module_id: str
    size: tuple[float, float, float]
    points: tuple[TrajectoryPoint, ...]
    collision_parts: tuple[CollisionPart, ...] = ()


@dataclass(frozen=True)
class TransitionValidationResult:
    feasible: bool
    reasons: tuple[str, ...]
    checked_samples: int


class TransitionTrajectoryValidator:
    """Conservative 3D validation after a fast 2D clearance check."""

    def __init__(self, temporal_resolution_s: float = 0.05,
                 ground_z: float = 0.0, support_tolerance_m: float = 0.01) -> None:
        if temporal_resolution_s <= 0:
            raise ValueError("temporal resolution must be positive")
        self.temporal_resolution_s = temporal_resolution_s
        self.ground_z = ground_z
        self.support_tolerance_m = support_tolerance_m

    def validate(
        self,
        trajectories: tuple[ModuleTrajectory, ...],
        static_modules: tuple[Box3, ...],
        environment: tuple[Box3, ...],
        target_poses: Mapping[str, Pose3],
        max_relative_speed: float = 0.25,
        max_covariance_trace: float = 0.015,
        check_sensing: bool = True,
    ) -> TransitionValidationResult:
        reasons: set[str] = set()
        checked = 0
        if not trajectories:
            return TransitionValidationResult(False, ("missing_trajectory",), 0)
        for trajectory in trajectories:
            if len(trajectory.points) < 2 or any(
                later.time_s <= earlier.time_s
                for earlier, later in zip(trajectory.points, trajectory.points[1:])
            ):
                reasons.add(f"{trajectory.module_id}:invalid_timing")
                continue
            sampled = _sample(trajectory, self.temporal_resolution_s)
            checked += len(sampled)
            for point in sampled:
                moving_parts = _trajectory_boxes(trajectory, point.pose)
                lowest = min(box.center[2] - box.size[2] / 2 for box in moving_parts)
                if lowest > self.ground_z + self.support_tolerance_m:
                    reasons.add(f"{trajectory.module_id}:unsupported")
                if any(_overlap(moving, obstacle) for moving in moving_parts
                       for obstacle in environment):
                    reasons.add(f"{trajectory.module_id}:environment_collision")
                if any(_overlap(moving, module) for moving in moving_parts
                       for module in static_modules
                       if module.name != trajectory.module_id):
                    reasons.add(f"{trajectory.module_id}:self_collision")
                if check_sensing and not point.connector_visible:
                    reasons.add(f"{trajectory.module_id}:connector_not_visible")
                if check_sensing and point.covariance_trace > max_covariance_trace:
                    reasons.add(f"{trajectory.module_id}:covariance_too_large")
            for first, second in zip(sampled, sampled[1:]):
                dt = second.time_s - first.time_s
                speed = hypot(second.pose.x - first.pose.x, second.pose.y - first.pose.y) / dt
                if speed > max_relative_speed + 1e-9:
                    reasons.add(f"{trajectory.module_id}:relative_velocity")
            final, target = sampled[-1], target_poses.get(trajectory.module_id)
            if target is None or _pose_distance(final.pose, target) > 0.025:
                reasons.add(f"{trajectory.module_id}:connector_reach")
            if not final.latch_confirmed:
                reasons.add(f"{trajectory.module_id}:latch_unconfirmed")
        # Moving trajectories may overlap one another at concurrent times.
        if len(trajectories) > 1:
            duration = max(t.points[-1].time_s for t in trajectories)
            step_count = int(duration / self.temporal_resolution_s) + 1
            for index in range(step_count + 1):
                timestamp = min(duration, index * self.temporal_resolution_s)
                boxes = [_trajectory_boxes(t, _at(t, timestamp).pose) for t in trajectories]
                if any(
                    _overlap(a, b)
                    for first_index, first in enumerate(boxes)
                    for second in boxes[first_index + 1:]
                    for a in first for b in second
                ):
                    reasons.add("moving_module_self_collision")
                    break
        return TransitionValidationResult(not reasons, tuple(sorted(reasons)), checked)

    def validate_environment(
        self,
        trajectories: tuple[ModuleTrajectory, ...],
        environment: tuple[Box3, ...],
    ) -> TransitionValidationResult:
        """Check only pose-dependent environment collisions for a cached edge."""
        reasons: set[str] = set()
        checked = 0
        for trajectory in trajectories:
            sampled = _sample(trajectory, self.temporal_resolution_s)
            checked += len(sampled)
            if any(
                _overlap(moving, obstacle)
                for point in sampled
                for moving in _trajectory_boxes(trajectory, point.pose)
                for obstacle in environment
            ):
                reasons.add(f"{trajectory.module_id}:environment_collision")
        return TransitionValidationResult(not reasons, tuple(sorted(reasons)), checked)

    def swept_boxes(
        self, trajectories: tuple[ModuleTrajectory, ...]
    ) -> tuple[Box3, ...]:
        """Return a deduplicated conservative swept-box representation."""
        output: list[Box3] = []
        seen: set[tuple] = set()
        for trajectory in trajectories:
            for point in _sample(trajectory, self.temporal_resolution_s):
                for box in _trajectory_boxes(trajectory, point.pose):
                    key = (
                        box.name,
                        *(round(value, 4) for value in box.center),
                        *(round(value, 4) for value in box.size),
                    )
                    if key not in seen:
                        seen.add(key)
                        output.append(box)
        return tuple(output)

    @staticmethod
    def validate_swept_environment(
        swept_boxes: tuple[Box3, ...],
        environment: tuple[Box3, ...],
        offset_x: float,
        offset_y: float,
    ) -> TransitionValidationResult:
        reasons: set[str] = set()
        for box in swept_boxes:
            translated = Box3(
                box.name,
                (box.center[0] + offset_x, box.center[1] + offset_y, box.center[2]),
                box.size,
            )
            if any(_overlap(translated, obstacle) for obstacle in environment):
                reasons.add(f"{box.name.split('/', 1)[0]}:environment_collision")
        return TransitionValidationResult(
            not reasons, tuple(sorted(reasons)), len(swept_boxes))


def load_environment_boxes(path: str) -> tuple[Box3, ...]:
    """Static 3D transition obstacles from a scenario manifest (prior map).

    An empty path means no 3D prior beyond the occupancy map.
    """
    if not path:
        return ()
    import json

    with open(path, encoding="utf-8") as stream:
        manifest = json.load(stream)
    return tuple(
        Box3(str(box["name"]), tuple(float(v) for v in box["center"]),
             tuple(float(v) for v in box["size"]))
        for box in manifest.get("transition_obstacles", ())
    )


def _sample(trajectory: ModuleTrajectory, resolution: float) -> tuple[TrajectoryPoint, ...]:
    end = trajectory.points[-1].time_s
    count = int(end / resolution)
    times = [index * resolution for index in range(count + 1)]
    if not times or times[-1] < end:
        times.append(end)
    return tuple(_at(trajectory, timestamp) for timestamp in times)


def _at(trajectory: ModuleTrajectory, timestamp: float) -> TrajectoryPoint:
    if timestamp <= trajectory.points[0].time_s:
        return trajectory.points[0]
    for first, second in zip(trajectory.points, trajectory.points[1:]):
        if timestamp <= second.time_s:
            fraction = (timestamp - first.time_s) / (second.time_s - first.time_s)
            pose = Pose3(
                first.pose.x + fraction * (second.pose.x - first.pose.x),
                first.pose.y + fraction * (second.pose.y - first.pose.y),
                first.pose.z + fraction * (second.pose.z - first.pose.z),
                first.pose.yaw + fraction * (second.pose.yaw - first.pose.yaw),
            )
            return TrajectoryPoint(timestamp, pose,
                first.connector_visible and second.connector_visible,
                max(first.covariance_trace, second.covariance_trace),
                second.latch_confirmed if fraction >= 1.0 else False)
    return trajectory.points[-1]


def _pose_box(name: str, pose: Pose3, size: tuple[float, float, float]) -> Box3:
    # Axis-aligned bounds of the rotated collision box are conservative.
    extent_x = abs(cos(pose.yaw)) * size[0] + abs(sin(pose.yaw)) * size[1]
    extent_y = abs(sin(pose.yaw)) * size[0] + abs(cos(pose.yaw)) * size[1]
    return Box3(name, (pose.x, pose.y, pose.z), (extent_x, extent_y, size[2]))


def _trajectory_boxes(trajectory: ModuleTrajectory, pose: Pose3) -> tuple[Box3, ...]:
    if not trajectory.collision_parts:
        return (_pose_box(trajectory.module_id, pose, trajectory.size),)
    output = []
    for part in trajectory.collision_parts:
        x, y, z = part.center
        center = Pose3(
            pose.x + cos(pose.yaw) * x - sin(pose.yaw) * y,
            pose.y + sin(pose.yaw) * x + cos(pose.yaw) * y,
            pose.z + z,
            pose.yaw,
        )
        output.append(_pose_box(f"{trajectory.module_id}/{part.name}", center, part.size))
    return tuple(output)


def _overlap(first: Box3, second: Box3) -> bool:
    return all(abs(a - b) < (sa + sb) / 2 - 1e-9
               for a, b, sa, sb in zip(first.center, second.center, first.size, second.size))


def _pose_distance(first: Pose3, second: Pose3) -> float:
    return ((first.x - second.x) ** 2 + (first.y - second.y) ** 2 +
            (first.z - second.z) ** 2) ** 0.5
