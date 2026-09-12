from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, pi, sin
from typing import Callable, Literal, Mapping

from .catalog import Catalog, Transition
from .grid import OccupancyGrid
from .planner import HybridState
from .transition_validation import (
    Box3,
    CollisionPart,
    ModuleTrajectory,
    Pose3,
    TrajectoryPoint,
    TransitionValidationResult,
    TransitionTrajectoryValidator,
)


CoupledMethod = Literal[
    "geometry_coupled",
    "feasibility_coupled",
    "sensing_feasibility_coupled",
]
SensingProvider = Callable[
    [Transition, HybridState], Mapping[str, "PodSensingState"]
]


@dataclass(frozen=True)
class PodSensingState:
    connector_visible: bool
    covariance_trace: float


@dataclass(frozen=True)
class TransitionDecision:
    method: CoupledMethod
    transition_id: str
    state: HybridState
    feasible: bool
    reasons: tuple[str, ...]
    checked_samples: int = 0

    def to_record(self) -> dict:
        return {
            "method": self.method,
            "transition_id": self.transition_id,
            "state": {
                "x": self.state.x, "y": self.state.y,
                "heading": self.state.heading,
                "morphology": self.state.morphology,
            },
            "feasible": self.feasible,
            "reasons": list(self.reasons),
            "checked_samples": self.checked_samples,
        }


def _wrap_yaw(value: float) -> float:
    return (value + pi) % (2.0 * pi) - pi


def _world_pose(relative: tuple[float, float, float], base: Pose3,
                module_height: float) -> Pose3:
    x, y, yaw = relative
    return Pose3(
        base.x + cos(base.yaw) * x - sin(base.yaw) * y,
        base.y + sin(base.yaw) * x + cos(base.yaw) * y,
        base.z + module_height / 2.0,
        _wrap_yaw(base.yaw + yaw),
    )


def build_transition_trajectories(
    catalog: Catalog,
    transition: Transition,
    base_pose: Pose3,
    sensing: Mapping[str, PodSensingState] | None = None,
) -> tuple[ModuleTrajectory, ...]:
    """Expand catalog waypoints into sequential, world-frame pod trajectories.

    Catalog poses are ``(x, y, yaw)`` in the core frame. Every trajectory spans
    the complete nominal transition duration: it holds the source pose until
    its pod's turn, follows its waypoints, and then holds the target pose. This
    makes collision checks across sequential pod moves physically meaningful.
    """
    source = catalog.morphologies[transition.source]
    target = catalog.morphologies[transition.target]
    moved = transition.moved_pods
    if not moved:
        raise ValueError(f"{transition.id}: transition has no moved pods")
    slot = transition.time / len(moved)
    trajectories: list[ModuleTrajectory] = []
    for pod_index, pod in enumerate(moved):
        if pod not in catalog.module_sizes:
            raise ValueError(f"{transition.id}: no collision size for {pod}")
        size = catalog.module_sizes[pod]
        relative_path = [source.pod_poses[pod], *transition.pod_waypoints.get(pod, ())]
        if not relative_path or relative_path[-1] != target.pod_poses[pod]:
            relative_path.append(target.pod_poses[pod])
        # Remove adjacent duplicates so generated timestamps remain strict.
        path = [relative_path[0]]
        path.extend(pose for pose in relative_path[1:] if pose != path[-1])
        if len(path) < 2:
            raise ValueError(f"{transition.id}: {pod} has no relocation path")
        weights = [
            max(1e-6, hypot(b[0] - a[0], b[1] - a[1]) + 0.05 * abs(_wrap_yaw(b[2] - a[2])))
            for a, b in zip(path, path[1:])
        ]
        move_start = pod_index * slot
        move_end = (pod_index + 1) * slot
        sensor = (sensing or {}).get(pod, PodSensingState(True, 0.0))
        points = [TrajectoryPoint(
            0.0, _world_pose(path[0], base_pose, size[2]),
            sensor.connector_visible, sensor.covariance_trace, False,
        )]
        if move_start > 0.0:
            points.append(TrajectoryPoint(
                move_start, _world_pose(path[0], base_pose, size[2]),
                sensor.connector_visible, sensor.covariance_trace, False,
            ))
        elapsed = move_start
        total_weight = sum(weights)
        for index, (pose, weight) in enumerate(zip(path[1:], weights)):
            elapsed += slot * weight / total_weight
            points.append(TrajectoryPoint(
                min(move_end, elapsed), _world_pose(pose, base_pose, size[2]),
                sensor.connector_visible, sensor.covariance_trace,
                index == len(weights) - 1,
            ))
        if points[-1].time_s < transition.time:
            final = points[-1]
            points.append(TrajectoryPoint(
                transition.time, final.pose, final.connector_visible,
                final.covariance_trace, True,
            ))
        collision_parts = tuple(
            CollisionPart(name, center, part_size)
            for name, center, part_size in catalog.module_collision_boxes.get(pod, ())
        )
        trajectories.append(ModuleTrajectory(pod, size, tuple(points), collision_parts))
    return tuple(trajectories)


class CoupledTransitionPolicy:
    """Method-specific transition predicate with structured audit records."""

    def __init__(
        self,
        method: CoupledMethod,
        catalog: Catalog,
        grid: OccupancyGrid,
        heading_bins: int,
        environment: tuple[Box3, ...] = (),
        static_modules: tuple[Box3, ...] = (),
        sensing: Mapping[str, PodSensingState] | None = None,
        sensing_provider: SensingProvider | None = None,
        trajectory_validator: TransitionTrajectoryValidator | None = None,
    ) -> None:
        if method not in {
            "geometry_coupled", "feasibility_coupled",
            "sensing_feasibility_coupled",
        }:
            raise ValueError(f"unsupported coupled method: {method}")
        self.method = method
        self.catalog = catalog
        self.grid = grid
        self.heading_bins = heading_bins
        self.environment = environment
        self.static_modules = static_modules
        self.sensing = sensing or {}
        self.sensing_provider = sensing_provider
        self.validator = trajectory_validator or TransitionTrajectoryValidator()
        self.decisions: list[TransitionDecision] = []
        self._intrinsic_cache: dict[str, TransitionValidationResult] = {}
        self._disk_cache: dict[tuple[int, int, float], bool] = {}
        self._target_cache: dict[tuple[int, int, int, str], bool] = {}
        self._swept_boxes_cache: dict[tuple[str, int], tuple[Box3, ...]] = {}

    def __call__(self, transition: Transition, state: HybridState) -> bool:
        reasons: list[str] = []
        wx, wy = self.grid.cell_center(state.x, state.y)
        disk_key = (state.x, state.y, transition.swept_radius)
        disk_free = self._disk_cache.get(disk_key)
        if disk_free is None:
            disk_free = self.grid.disk_is_free(wx, wy, transition.swept_radius)
            self._disk_cache[disk_key] = disk_free
        if not disk_free:
            reasons.append("planar_clearance")
        target = HybridState(state.x, state.y, state.heading, transition.target)
        morphology = self.catalog.morphologies[target.morphology]
        yaw = state.heading * 2.0 * pi / self.heading_bins
        target_key = (state.x, state.y, state.heading, transition.target)
        target_free = self._target_cache.get(target_key)
        if target_free is None:
            target_free = self.grid.footprint_is_free(
                wx, wy, yaw, morphology.footprint)
            self._target_cache[target_key] = target_free
        if not target_free:
            reasons.append("target_footprint")
        checked = 0
        if not reasons and self.method != "geometry_coupled":
            sensing = None
            if self.method == "sensing_feasibility_coupled":
                sensing = (
                    self.sensing_provider(transition, state)
                    if self.sensing_provider else self.sensing
                )
            if self.static_modules:
                trajectories = build_transition_trajectories(
                    self.catalog, transition, Pose3(wx, wy, 0.0, yaw), sensing)
                targets = self._target_poses(transition, Pose3(wx, wy, 0.0, yaw))
                validation = self.validator.validate(
                    trajectories, self.static_modules, self.environment, targets,
                    check_sensing=False,
                )
                reasons.extend(validation.reasons)
                checked = validation.checked_samples
            else:
                intrinsic = self._intrinsic_cache.get(transition.id)
                if intrinsic is None:
                    trajectories = build_transition_trajectories(
                        self.catalog, transition, Pose3(0.0, 0.0, 0.0, 0.0))
                    targets = self._target_poses(transition, Pose3(0.0, 0.0, 0.0, 0.0))
                    intrinsic = self.validator.validate(
                        trajectories, (), (), targets, check_sensing=False)
                    self._intrinsic_cache[transition.id] = intrinsic
                reasons.extend(intrinsic.reasons)
                checked = intrinsic.checked_samples
                nearby = tuple(
                    obstacle for obstacle in self.environment
                    if abs(obstacle.center[0] - wx) <= transition.swept_radius + obstacle.size[0] / 2
                    and abs(obstacle.center[1] - wy) <= transition.swept_radius + obstacle.size[1] / 2
                )
                if not reasons and nearby:
                    sweep_key = (transition.id, state.heading)
                    swept = self._swept_boxes_cache.get(sweep_key)
                    if swept is None:
                        local = build_transition_trajectories(
                            self.catalog, transition, Pose3(0.0, 0.0, 0.0, yaw))
                        swept = self.validator.swept_boxes(local)
                        self._swept_boxes_cache[sweep_key] = swept
                    environment_result = self.validator.validate_swept_environment(
                        swept, nearby, wx, wy)
                    reasons.extend(environment_result.reasons)
                    checked += environment_result.checked_samples
            if self.method == "sensing_feasibility_coupled":
                for pod in transition.moved_pods:
                    evidence = sensing.get(pod, PodSensingState(False, float("inf")))
                    if not evidence.connector_visible:
                        reasons.append(f"{pod}:connector_not_visible")
                    if evidence.covariance_trace > 0.015:
                        reasons.append(f"{pod}:covariance_too_large")
        decision = TransitionDecision(
            self.method, transition.id, state, not reasons, tuple(sorted(set(reasons))), checked,
        )
        self.decisions.append(decision)
        return decision.feasible

    def _target_poses(self, transition: Transition, base: Pose3) -> dict[str, Pose3]:
        return {
            pod: _world_pose(
                self.catalog.morphologies[transition.target].pod_poses[pod],
                base, self.catalog.module_sizes[pod][2],
            )
            for pod in transition.moved_pods
        }
