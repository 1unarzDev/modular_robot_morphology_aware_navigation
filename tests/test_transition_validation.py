from morphology_planner.transition_validation import (
    Box3, ModuleTrajectory, Pose3, TrajectoryPoint, TransitionTrajectoryValidator,
)


def trajectory(**changes):
    points = (
        TrajectoryPoint(0.0, Pose3(0.0, 0.0, 0.05), covariance_trace=0.001),
        TrajectoryPoint(2.0, Pose3(0.4, 0.0, 0.05), covariance_trace=0.001,
                        latch_confirmed=True),
    )
    value = {"module_id": "pod_0", "size": (0.18, 0.10, 0.10), "points": points}
    value.update(changes)
    return ModuleTrajectory(**value)


def test_valid_3d_transition_is_continuously_sampled():
    result = TransitionTrajectoryValidator(temporal_resolution_s=0.05).validate(
        (trajectory(),), (), (), {"pod_0": Pose3(0.4, 0.0, 0.05)})
    assert result.feasible and result.checked_samples == 41


def test_transition_can_pass_planar_clearance_but_fail_3d_overhead_collision():
    # The low overhang is not a ground-plane obstacle, so a 2D disk precheck can
    # pass while the module's actual collision box intersects it.
    overhang = Box3("overhang", (0.2, 0.0, 0.11), (0.12, 0.3, 0.06))
    result = TransitionTrajectoryValidator().validate(
        (trajectory(),), (), (overhang,), {"pod_0": Pose3(0.4, 0.0, 0.05)})
    assert not result.feasible
    assert "pod_0:environment_collision" in result.reasons


def test_validator_reports_support_visibility_uncertainty_velocity_and_latch():
    bad = ModuleTrajectory("pod_0", (0.18, 0.10, 0.10), (
        TrajectoryPoint(0.0, Pose3(0.0, 0.0, 0.20), False, 0.02),
        TrajectoryPoint(0.1, Pose3(0.4, 0.0, 0.20), False, 0.02, False),
    ))
    result = TransitionTrajectoryValidator().validate(
        (bad,), (), (), {"pod_0": Pose3(0.5, 0.0, 0.20)})
    assert not result.feasible
    assert {
        "pod_0:unsupported", "pod_0:connector_not_visible",
        "pod_0:covariance_too_large", "pod_0:relative_velocity",
        "pod_0:connector_reach", "pod_0:latch_unconfirmed",
    } <= set(result.reasons)
