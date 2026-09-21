from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_ros_consumers_use_estimator_owned_sensing_revision() -> None:
    sources = (
        ROOT / "src/modular_robot_bringup/modular_robot_bringup/hybrid_navigator.py",
        ROOT / "src/morphology_planner/morphology_planner/ros_node.py",
    )
    for source in sources:
        text = source.read_text(encoding="utf-8")
        callback = text.split("def _on_sensing", 1)[1].split("\n    def ", 1)[0]
        assert "message.sensing_revision" in callback
        assert "sensing_revision += 1" not in callback


def test_navigator_retries_preplan_revision_changes() -> None:
    source = ROOT / "src/modular_robot_bringup/modular_robot_bringup/hybrid_navigator.py"
    text = source.read_text(encoding="utf-8")
    assert '"changed before planning" in plan_result.message' in text


def test_docking_stability_counts_fresh_samples_not_planning_revisions() -> None:
    source = ROOT / "src/reconfiguration_executor/reconfiguration_executor/node.py"
    text = source.read_text(encoding="utf-8")
    callback = text.split("async def _wait_for_docking_ready", 1)[1].split(
        "\n    async def ", 1)[0]
    assert "estimate.timestamp > previous_timestamp" in callback
    assert "estimate.sensing_revision !=" not in callback


def test_transition_success_waits_for_target_morphology_acknowledgement() -> None:
    source = ROOT / "src/modular_robot_bringup/modular_robot_bringup/hybrid_navigator.py"
    text = source.read_text(encoding="utf-8")
    callback = text.split("async def _execute_transition", 1)[1].split(
        "\n    @staticmethod", 1)[0]
    assert "result.resulting_morphology" in callback
    assert "MorphologyState.READY" in callback
    assert "topology_revision) > starting_revision" in callback


def test_assembled_motion_command_window_is_measured_in_simulated_time() -> None:
    """A wall-clock window scales commanded travel with the real-time factor.

    `time.monotonic()` may bound the stage only as a stall backstop, which is
    what `CommandWindow` uses it for.
    """
    source = ROOT / "src/modular_robot_bringup/modular_robot_bringup/hybrid_navigator.py"
    text = source.read_text(encoding="utf-8")
    body = text.split("async def _command_for", 1)[1].split(
        "\n    async def ", 1)[0]
    assert "CommandWindow(" in body
    assert "self.get_clock()" in body
    assert "deadline = time.monotonic()" not in body


def test_drive_command_staleness_is_measured_on_the_robot_clock() -> None:
    """A wall-clock staleness threshold expires against a simulated-time
    publisher once the real-time factor falls below `period / timeout`, zeroing
    a live command mid-maneuver. This is the same defect as the assembled
    motion command window, in the node that actuates it."""
    source = ROOT / "src/modular_robot_bringup/modular_robot_bringup/drive_adapter.py"
    text = source.read_text(encoding="utf-8")
    assert "import time" not in text
    assert "time.monotonic()" not in text
    assert "self.get_clock().now()" in text
