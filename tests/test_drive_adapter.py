from modular_robot_bringup.kinematics import BodyTwist, distribute_twist


def test_forward_body_command_reaches_forward_facing_pods():
    commands = distribute_twist(
        BodyTwist(x=0.7),
        {"left": [0.2, 0.2, 0.0], "right": [0.2, -0.2, 0.0]},
        1.0,
    )
    assert commands["left"].linear == 0.7
    assert commands["right"].linear == 0.7


def test_yaw_command_creates_differential_array_speeds():
    commands = distribute_twist(
        BodyTwist(yaw=1.0),
        {"left": [0.0, 0.25, 0.0], "right": [0.0, -0.25, 0.0]},
        1.0,
    )
    assert commands["left"].linear == -0.25
    assert commands["right"].linear == 0.25


def test_orthogonal_pods_generate_lateral_motion_and_clip():
    commands = distribute_twist(
        BodyTwist(y=2.0),
        {
            "north": [0.0, 0.2, 1.57079632679],
            "south": [0.0, -0.2, -1.57079632679],
        },
        0.8,
    )
    assert commands["north"].linear == 0.8
    assert commands["south"].linear == -0.8


def test_yaw_effort_scale_increases_tangent_speed_for_narrow_track():
    commands = distribute_twist(
        BodyTwist(yaw=0.25),
        {"left": [0.0, 0.10, 0.0], "right": [0.0, -0.10, 0.0]},
        1.0, yaw_effort_scale=3.0,
    )
    assert abs(commands["left"].linear + 0.075) < 1e-12
    assert abs(commands["right"].linear - 0.075) < 1e-12
