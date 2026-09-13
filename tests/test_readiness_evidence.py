from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_controller_readiness_uses_lifecycle_transition_events_with_poll_fallback():
    source = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/mission_trial.py").read_text()
    assert '"/controller_server/transition_event"' in source
    assert "message.goal_state.id == State.PRIMARY_STATE_ACTIVE" in source
    assert "self._poll_controller_state" in source


def test_readiness_timeout_reports_each_gate():
    source = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/mission_trial.py").read_text()
    for gate in ("clock", "morphology", "map", "navigate_hybrid_action",
                 "controller_active", "recent_odom", "recent_scan",
                 "map_to_base_transform"):
        assert f'"{gate}"' in source
