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


def _final_alignment_converges(start: Pose2, terminal_gain: float, budget_s: float = 45.0):
    """Integrate a unicycle pod under the executor's final-alignment call."""
    from math import cos, sin
    target, pose, dt = Pose2(0.35, 0.20, 0.0), start, 0.05
    for _ in range(int(budget_s / dt)):
        command, _ = docking_command(pose, target, 0.006, pi, 0.08, 0.9,
                                     terminal_gain=terminal_gain)
        if ((pose.x - target.x) ** 2 + (pose.y - target.y) ** 2) ** 0.5 <= 0.006:
            return True
        pose = Pose2(pose.x + command.linear * cos(pose.yaw) * dt,
                     pose.y + command.linear * sin(pose.yaw) * dt,
                     wrap_angle(pose.yaw + command.angular * dt))
    return False


# pod_4's recorded poses before and at the stall in fault-matrix trial 00-r06.
@pytest.mark.parametrize("start", [Pose2(0.3771, 0.2055, -1.77),
                                   Pose2(0.3705, 0.1977, -2.1767)])
def test_final_alignment_has_no_stall_just_past_the_target(start):
    # With the terminal yaw coupled in, a pod that overshot the target parks
    # outside the linear gate: no drive, and a turn command near zero.
    command, _ = docking_command(Pose2(0.3705, 0.1977, -2.1767), Pose2(0.35, 0.20, 0.0),
                                 0.006, pi, 0.08, 0.9)
    assert command.linear == 0.0 and abs(command.angular) < 0.05
    assert _final_alignment_converges(start, terminal_gain=0.0)


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
