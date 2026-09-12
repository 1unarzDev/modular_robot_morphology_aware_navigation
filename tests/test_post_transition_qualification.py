from modular_robot_bringup.qualification import (
    PlanarPose, motion_delta, qualification_pass,
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
