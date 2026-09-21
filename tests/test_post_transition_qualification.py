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


def _run_window(duration, real_time_factor, stall_grace_s=30.0, tick_s=0.05,
                stall_after_s=None):
    """Drive one command window with a simulated clock at a fixed rate.

    Returns the simulated seconds the stage was commanded for, or None if the
    window reported a stalled simulated clock.
    """
    from modular_robot_bringup.qualification import CommandWindow

    wall = [0.0]
    simulated = [0.0]
    window = CommandWindow(duration, lambda: simulated[0], lambda: wall[0],
                           stall_grace_s)
    while window.keep_commanding():
        if window.stalled:
            return None
        wall[0] += tick_s
        if stall_after_s is None or wall[0] < stall_after_s:
            simulated[0] += tick_s * real_time_factor
    return window.elapsed_s


def test_command_window_delivers_full_motion_when_simulation_lags():
    """The gate must measure mechanics, not host load.

    The published Neutral `geometry_coupled` failure ran the four-stage
    maneuver for 5.00 s of simulated time where every passing repeat ran
    6.00 s, because the window was bounded by `time.monotonic()` while the
    robot moved in simulated time. Travel then scales with the real-time
    factor: that run reached 0.0813 m forward and -0.0788 m reverse against a
    -0.08 m floor and a 0.113 m norm, having struck nothing.
    """
    for real_time_factor in (1.0, 0.94, 0.68, 0.25):
        assert _run_window(1.0, real_time_factor) >= 1.0
        assert _run_window(1.5, real_time_factor) >= 1.5


def test_command_window_reports_a_stalled_simulated_clock():
    """A simulated clock that stops must not block until the trial watchdog."""
    assert _run_window(1.0, 1.0, stall_grace_s=2.0, stall_after_s=0.5) is None


def test_command_window_does_not_stall_on_a_merely_slow_simulator():
    assert _run_window(1.0, 0.1, stall_grace_s=30.0) >= 1.0
