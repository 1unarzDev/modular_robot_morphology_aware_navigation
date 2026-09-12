from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from math import atan2, cos, hypot, sin

from .control import Pose2, wrap_angle


class UncertaintyClass(IntEnum):
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    UNOBSERVED = 3


@dataclass(frozen=True)
class RelativePoseObservation:
    pod_id: str
    source: str
    timestamp: float
    pose: Pose2
    variance_x: float
    variance_y: float
    variance_yaw: float
    visible: bool = True

    def validate(self) -> None:
        if self.source not in {"wheel_odometry", "imu", "fiducial"}:
            raise ValueError(f"unsupported sensor source: {self.source}")
        if min(self.variance_x, self.variance_y, self.variance_yaw) <= 0:
            raise ValueError("observation variances must be positive")


@dataclass(frozen=True)
class RelativePoseEstimate:
    pod_id: str
    timestamp: float
    pose: Pose2
    variance_x: float
    variance_y: float
    variance_yaw: float
    uncertainty_class: UncertaintyClass
    visible: bool
    sources: tuple[str, ...]
    sensing_revision: int

    @property
    def covariance_trace(self) -> float:
        return self.variance_x + self.variance_y + self.variance_yaw


@dataclass(frozen=True)
class DockingEvidence:
    estimate: RelativePoseEstimate
    target: Pose2
    relative_linear_speed: float
    relative_angular_speed: float
    approach_alignment: float
    latch_confirmed: bool


class RelativePoseEstimator:
    """Information-weighted sensor fusion without simulator world pose input."""

    def __init__(self, stale_after_s: float = 0.5) -> None:
        self.stale_after_s = stale_after_s
        self._observations: dict[tuple[str, str], RelativePoseObservation] = {}
        self.sensing_revision = 0

    def update(self, observation: RelativePoseObservation) -> int:
        observation.validate()
        key = (observation.pod_id, observation.source)
        previous = self._observations.get(key)
        if previous is not None and observation.timestamp <= previous.timestamp:
            raise ValueError("stale or out-of-order sensor observation")
        self._observations[key] = observation
        self.sensing_revision += 1
        return self.sensing_revision

    def estimate(self, pod_id: str, now: float) -> RelativePoseEstimate:
        observations = [
            value for (pod, _), value in self._observations.items()
            if pod == pod_id and now - value.timestamp <= self.stale_after_s
        ]
        if not observations:
            return RelativePoseEstimate(
                pod_id, now, Pose2(0.0, 0.0, 0.0), float("inf"), float("inf"),
                float("inf"), UncertaintyClass.UNOBSERVED, False, (), self.sensing_revision,
            )
        x, variance_x = _fuse_scalar([(value.pose.x, value.variance_x) for value in observations])
        y, variance_y = _fuse_scalar([(value.pose.y, value.variance_y) for value in observations])
        yaw, variance_yaw = _fuse_angle([(value.pose.yaw, value.variance_yaw) for value in observations])
        trace = variance_x + variance_y + variance_yaw
        uncertainty = (
            UncertaintyClass.LOW if trace <= 0.003
            else UncertaintyClass.MEDIUM if trace <= 0.015
            else UncertaintyClass.HIGH
        )
        return RelativePoseEstimate(
            pod_id, max(value.timestamp for value in observations), Pose2(x, y, yaw),
            variance_x, variance_y, variance_yaw, uncertainty,
            any(value.source == "fiducial" and value.visible for value in observations),
            tuple(sorted(value.source for value in observations)), self.sensing_revision,
        )


def docking_acceptance(
    evidence: DockingEvidence,
    translation_tolerance: float = 0.025,
    yaw_tolerance: float = 0.08,
    max_covariance_trace: float = 0.015,
    max_linear_speed: float = 0.02,
    max_angular_speed: float = 0.08,
    minimum_approach_alignment: float = 0.9,
) -> tuple[bool, tuple[str, ...]]:
    reasons = []
    estimate = evidence.estimate
    if estimate.uncertainty_class == UncertaintyClass.UNOBSERVED:
        reasons.append("missing_observation")
    if not estimate.visible:
        reasons.append("connector_not_visible")
    if estimate.covariance_trace > max_covariance_trace:
        reasons.append("covariance_too_large")
    if hypot(estimate.pose.x - evidence.target.x, estimate.pose.y - evidence.target.y) > translation_tolerance:
        reasons.append("translation_tolerance")
    if abs(wrap_angle(estimate.pose.yaw - evidence.target.yaw)) > yaw_tolerance:
        reasons.append("yaw_tolerance")
    if evidence.relative_linear_speed > max_linear_speed or evidence.relative_angular_speed > max_angular_speed:
        reasons.append("relative_velocity")
    if evidence.approach_alignment < minimum_approach_alignment:
        reasons.append("approach_direction")
    if not evidence.latch_confirmed:
        reasons.append("latch_unconfirmed")
    return not reasons, tuple(reasons)


def relative_pose(core: Pose2, pod: Pose2) -> Pose2:
    """Express an odometric pod pose in the core frame."""
    dx, dy = pod.x - core.x, pod.y - core.y
    return Pose2(cos(core.yaw) * dx + sin(core.yaw) * dy,
                 -sin(core.yaw) * dx + cos(core.yaw) * dy,
                 wrap_angle(pod.yaw - core.yaw))


def anchored_odometry(anchor_relative: Pose2, baseline: Pose2, current: Pose2) -> Pose2:
    """Apply local wheel-odometry displacement to a topology-derived anchor."""
    displacement = relative_pose(baseline, current)
    return Pose2(
        anchor_relative.x + cos(anchor_relative.yaw) * displacement.x
        - sin(anchor_relative.yaw) * displacement.y,
        anchor_relative.y + sin(anchor_relative.yaw) * displacement.x
        + cos(anchor_relative.yaw) * displacement.y,
        wrap_angle(anchor_relative.yaw + displacement.yaw),
    )


def _fuse_scalar(values: list[tuple[float, float]]) -> tuple[float, float]:
    information = sum(1.0 / variance for _, variance in values)
    return sum(value / variance for value, variance in values) / information, 1.0 / information


def _fuse_angle(values: list[tuple[float, float]]) -> tuple[float, float]:
    information = sum(1.0 / variance for _, variance in values)
    sine = sum(sin(value) / variance for value, variance in values)
    cosine = sum(cos(value) / variance for value, variance in values)
    return wrap_angle(atan2(sine, cosine)), 1.0 / information
