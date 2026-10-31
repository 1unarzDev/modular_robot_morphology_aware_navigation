from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def test_narrow_controller_does_not_rotate_for_localization_scale_errors():
    config = yaml.safe_load(
        (ROOT / "src/modular_robot_bringup/config/nav2.yaml").read_text()
    )
    narrow = config["controller_server"]["ros__parameters"]["FollowPathNarrow"]
    assert narrow["use_rotate_to_heading"] is True
    assert narrow["rotate_to_heading_min_angle"] >= 0.15
    assert narrow["desired_linear_vel"] <= 0.2
    assert narrow["use_velocity_scaled_lookahead_dist"] is False
    # The lateral offset RPP tolerates before turning in place is the lookahead
    # times tan(min angle): it must exceed centimetre-scale localization
    # corrections (0.01 rad hunted), yet a short lookahead is what lets the
    # narrow body turn at planned vertices rather than cut them.
    from math import tan
    tolerated = narrow["lookahead_dist"] * tan(narrow["rotate_to_heading_min_angle"])
    assert tolerated >= 0.06
    goal_checker = config["controller_server"]["ros__parameters"]["goal_checker"]
    assert goal_checker["stateful"] is False
    assert goal_checker["yaw_goal_tolerance"] >= 3.14


def test_narrow_rotation_cannot_ratchet_up_on_odometry():
    """RPP clamps rotation to max_angular_accel*dt of measured rate; a narrow
    window let the narrow body's odometry over-report drive a runaway spin."""
    import yaml
    from pathlib import Path

    nav2 = yaml.safe_load((Path(__file__).resolve().parents[1]
                           / "src/modular_robot_bringup/config/nav2.yaml").read_text())
    params = nav2["controller_server"]["ros__parameters"]
    narrow = params["FollowPathNarrow"]
    window = narrow["max_angular_accel"] / params["controller_frequency"]
    # Wider than any rate the body can report (odometry saturated near 1.6).
    assert window >= 1.6 - narrow["rotate_to_heading_angular_vel"]
