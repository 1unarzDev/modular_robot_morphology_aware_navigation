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
