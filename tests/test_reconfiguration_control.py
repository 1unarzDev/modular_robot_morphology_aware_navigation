from math import pi

import pytest

from reconfiguration_executor.control import (
    INJECTABLE_FAILURE_STAGES, Pose2, compose_pose, docking_command,
    injected_failure, wrap_angle,
)


def test_pose_composition_respects_core_heading():
    result = compose_pose(Pose2(2.0, 3.0, pi / 2), [0.4, 0.1, -pi / 2])
    assert abs(result.x - 1.9) < 1e-9
    assert abs(result.y - 3.4) < 1e-9
    assert abs(result.yaw) < 1e-9


def test_docking_uses_reverse_for_target_behind():
    command, arrived = docking_command(Pose2(0, 0, 0), Pose2(-1, 0, 0), 0.02, 0.05, 0.2, 1.0)
    assert not arrived
    assert command.linear == -0.2
    assert abs(command.angular) < 1e-9


def test_side_target_rotates_without_forward_reverse_chatter():
    command, arrived = docking_command(Pose2(0, 0, 0), Pose2(0, -1, 0), 0.02, 0.05, 0.2, 1.0)
    assert not arrived
    assert command.linear == 0.0
    assert command.angular == -1.0


def test_docking_has_final_yaw_phase_and_terminal_state():
    command, arrived = docking_command(Pose2(1, 1, 0), Pose2(1, 1, 0.2), 0.02, 0.05, 0.2, 1.0)
    assert not arrived and command.angular > 0
    command, arrived = docking_command(Pose2(1, 1, 0.18), Pose2(1, 1, 0.2), 0.02, 0.05, 0.2, 1.0)
    assert arrived and command.linear == command.angular == 0.0


def test_angle_wrap_is_symmetric_at_pi():
    assert abs(wrap_angle(3 * pi) + pi) < 1e-9


@pytest.mark.parametrize("stage", sorted(INJECTABLE_FAILURE_STAGES))
def test_failure_injection_is_deterministic_for_every_stage(stage):
    assert injected_failure(stage, stage)
    assert injected_failure(f"{stage}:pod_2", stage, "pod_2")
    assert not injected_failure(f"{stage}:pod_2", stage, "pod_1")


def test_unknown_failure_injection_is_rejected():
    with pytest.raises(ValueError, match="unknown failure"):
        injected_failure("typo", "detach")
