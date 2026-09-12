from math import pi

import pytest

from reconfiguration_executor.control import Pose2
from reconfiguration_executor.sensing import (
    DockingEvidence, RelativePoseEstimator, RelativePoseObservation,
    UncertaintyClass, anchored_odometry, docking_acceptance, relative_pose,
)


def observation(source, timestamp, x=0.1, variance=0.004, visible=True):
    return RelativePoseObservation("pod_0", source, timestamp, Pose2(x, 0.0, 0.02),
                                   variance, variance, variance, visible)


def test_sensor_fusion_reduces_covariance_and_increments_revision():
    estimator = RelativePoseEstimator()
    assert estimator.update(observation("wheel_odometry", 1.0, x=0.09)) == 1
    first = estimator.estimate("pod_0", 1.1)
    estimator.update(observation("fiducial", 1.05, x=0.11, variance=0.001))
    fused = estimator.estimate("pod_0", 1.1)
    assert 0.09 < fused.pose.x < 0.11
    assert fused.covariance_trace < first.covariance_trace
    assert fused.visible and fused.sensing_revision == 2


def test_stale_and_missing_observations_are_explicit():
    estimator = RelativePoseEstimator(stale_after_s=0.2)
    estimator.update(observation("imu", 1.0))
    assert estimator.estimate("pod_0", 1.3).uncertainty_class == UncertaintyClass.UNOBSERVED
    with pytest.raises(ValueError, match="stale"):
        estimator.update(observation("imu", 0.9))


def test_latch_reset_discards_pre_seating_observations():
    estimator = RelativePoseEstimator()
    estimator.update(observation("wheel_odometry", 1.0, x=0.08))
    estimator.update(observation("fiducial", 1.0, x=0.12))
    assert estimator.reset_pod("pod_0") == 3
    assert estimator.estimate(
        "pod_0", 1.1).uncertainty_class == UncertaintyClass.UNOBSERVED
    estimator.update(observation("wheel_odometry", 1.2, x=0.10))
    assert estimator.estimate("pod_0", 1.2).pose.x == 0.10


def test_docking_gate_requires_sensing_motion_approach_and_latch():
    estimator = RelativePoseEstimator()
    estimator.update(observation("fiducial", 1.0, x=0.1, variance=0.0001))
    estimate = estimator.estimate("pod_0", 1.1)
    accepted, reasons = docking_acceptance(DockingEvidence(
        estimate, Pose2(0.1, 0.0, 0.02), 0.0, 0.0, 1.0, True))
    assert accepted and not reasons
    accepted, reasons = docking_acceptance(DockingEvidence(
        estimate, Pose2(0.1, 0.0, 0.02), 0.1, 0.2, 0.2, False))
    assert not accepted
    assert {"relative_velocity", "approach_direction", "latch_unconfirmed"} <= set(reasons)


def test_world_odometry_is_transformed_into_core_relative_frame():
    result = relative_pose(Pose2(1.0, 2.0, pi / 2), Pose2(1.0, 2.4, pi / 2))
    assert abs(result.x - 0.4) < 1e-9
    assert abs(result.y) < 1e-9
    assert abs(result.yaw) < 1e-9


def test_local_wheel_displacement_is_applied_to_topology_anchor():
    result = anchored_odometry(
        Pose2(0.23, 0.18, 0.0), Pose2(0.0, 0.0, 0.0), Pose2(0.15, -0.06, 0.1)
    )
    assert abs(result.x - 0.38) < 1e-9
    assert abs(result.y - 0.12) < 1e-9
    assert abs(result.yaw - 0.1) < 1e-9
