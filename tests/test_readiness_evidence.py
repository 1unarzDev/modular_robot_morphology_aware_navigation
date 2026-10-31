from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_controller_readiness_uses_lifecycle_transition_events_with_poll_fallback():
    source = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/mission_trial.py").read_text()
    assert '"/controller_server/transition_event"' in source
    assert "message.goal_state.id == State.PRIMARY_STATE_ACTIVE" in source
    assert "self._poll_controller_state" in source
    assert "latest_local_costmap_wall_time" in source


def test_readiness_timeout_reports_each_gate():
    source = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/mission_trial.py").read_text()
    for gate in ("clock", "morphology", "map", "navigate_hybrid_action",
                 "controller_active", "recent_odom", "recent_scan",
                 "map_to_base_transform"):
        assert f'"{gate}"' in source


def test_readiness_requires_the_navigation_lifecycle_manager_active():
    """A failed activation stops the manager before velocity_smoother and
    collision_monitor; controller_server alone looked active and the mission
    ran with no command reaching a wheel (pilot_v3, 2026-10-31)."""
    source = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/mission_trial.py").read_text()
    assert 'LIFECYCLE_MANAGERS = ("lifecycle_manager_navigation",)' in source
    assert 'f"/{name}/is_active"' in source
    assert '"lifecycle_managers_active"' in source
    assert "self._poll_lifecycle_managers" in source


def test_pod_drive_diagnostics_cover_a_mission_run_to_its_deadline():
    """Samples start at simulator start, before readiness and the goal, so a
    cap of exactly 300 s at 6 pods x 2 Hz dropped the first 50 s of evaluator
    truth from every timeout (narrow turn-rate experiment, 2026-10-31)."""
    import re

    source = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/mission_trial.py").read_text()
    cap = int(re.search(r"^POD_DRIVE_DIAGNOSTIC_CAP = (\d+)$", source, re.M).group(1))
    pods, rate_hz, deadline_s, pre_mission_s = 6, 2, 300.0, 30.0
    assert cap >= pods * rate_hz * (deadline_s + pre_mission_s)
    assert "len(self.pod_drive_diagnostics) > POD_DRIVE_DIAGNOSTIC_CAP" in source
