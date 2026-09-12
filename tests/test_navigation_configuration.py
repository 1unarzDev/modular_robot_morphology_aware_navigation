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
    assert narrow["lookahead_dist"] >= 0.8
    goal_checker = config["controller_server"]["ros__parameters"]["goal_checker"]
    assert goal_checker["stateful"] is False
    assert goal_checker["yaw_goal_tolerance"] >= 3.14
