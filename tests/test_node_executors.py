from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
POLLING_NODES = (
    ROOT / "src/modular_robot_bringup/modular_robot_bringup/hybrid_navigator.py",
    ROOT / "src/reconfiguration_executor/reconfiguration_executor/node.py",
)


def test_polling_nodes_use_single_threaded_executor() -> None:
    # Their 20-50 Hz coroutine polling cost roughly 3x more CPU per wake under
    # rclpy's multithreaded executor, pushing trials below real time.
    for source in POLLING_NODES:
        text = source.read_text(encoding="utf-8")
        assert "MultiThreadedExecutor" not in text
        assert "executor = SingleThreadedExecutor()" in text


def test_polling_node_blocking_waits_are_bounded() -> None:
    # A single thread must never wait indefinitely inside a callback.
    for source in POLLING_NODES:
        text = source.read_text(encoding="utf-8")
        assert "time.sleep(" not in text
        assert ".call(" not in text
        assert "spin_until_future_complete" not in text
        for line in text.splitlines():
            if "wait_for_service(" in line or "wait_for_server(" in line:
                assert "timeout_sec=" in line
