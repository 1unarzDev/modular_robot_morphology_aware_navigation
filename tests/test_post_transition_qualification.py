from modular_robot_bringup.qualification import (
    PlanarPose, goal_position_reached, motion_delta, qualification_pass,
)


def test_motion_delta_is_expressed_in_start_body_frame():
    delta = motion_delta(
        PlanarPose(0.0, 0.0, 1.57079632679),
        PlanarPose(0.0, 0.2, 1.57079632679),
    )
    assert delta["forward_m"] > 0.19
    assert abs(delta["lateral_m"]) < 1e-6


def test_qualification_fails_yaw_that_translates_instead_of_turning():
    stages = {
        "positive_yaw": {"yaw_rad": 0.02, "translation_m": 0.4},
        "negative_yaw": {"yaw_rad": -0.02, "translation_m": 0.4},
        "forward": {"forward_m": 0.12, "lateral_m": 0.0, "yaw_rad": 0.0},
        "reverse": {"forward_m": -0.12, "lateral_m": 0.0, "yaw_rad": 0.0},
    }
    assert not qualification_pass(stages)


def test_qualification_accepts_signed_motion_with_bounded_cross_coupling():
    stages = {
        "positive_yaw": {"yaw_rad": 0.2, "translation_m": 0.01},
        "negative_yaw": {"yaw_rad": -0.2, "translation_m": 0.01},
        "forward": {"forward_m": 0.11, "lateral_m": 0.01, "yaw_rad": 0.01},
        "reverse": {"forward_m": -0.11, "lateral_m": -0.01, "yaw_rad": -0.01},
    }
    assert qualification_pass(stages)


def test_qualification_uses_yaw_normalized_translation_coupling():
    stages = {
        "positive_yaw": {"yaw_rad": 0.4, "translation_m": 0.09},
        "negative_yaw": {"yaw_rad": -0.4, "translation_m": 0.09},
        "forward": {"forward_m": 0.11, "lateral_m": 0.0, "yaw_rad": 0.0},
        "reverse": {"forward_m": -0.11, "lateral_m": 0.0, "yaw_rad": 0.0},
    }
    assert qualification_pass(stages)
    stages["positive_yaw"] = {"yaw_rad": 0.2, "translation_m": 0.09}
    assert not qualification_pass(stages)


def test_terminal_position_check_rejects_goal_checker_drift():
    goal = PlanarPose(5.05, 1.75, 0.0)
    assert goal_position_reached(PlanarPose(4.95, 1.78, 1.0), goal, 0.15)
    assert not goal_position_reached(PlanarPose(4.87, 1.19, 1.4), goal, 0.15)


def test_executed_sequence_matches_the_planned_verification_declaration():
    """The planner sweeps the catalog declaration; the navigator executes this.

    If the two drift, the planner would accept sites against a maneuver the
    robot does not actually perform, which is the failure the transition model
    was extended to cover.
    """
    from modular_robot_bringup.qualification import VERIFICATION_SEQUENCE
    from morphology_planner import load_catalog

    catalog = load_catalog(
        "src/modular_robot_description/config/morphologies.yaml")
    declared = tuple(
        (stage.name, stage.linear, stage.angular, stage.duration)
        for stage in catalog.post_transition_verification.stages
    )
    assert declared == VERIFICATION_SEQUENCE
    assert set(name for name, *_ in VERIFICATION_SEQUENCE) == {
        "positive_yaw", "negative_yaw", "forward", "reverse"}
