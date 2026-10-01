"""The trial's backstops must be reachable when the executor will not yield.

Regression cover for the fault that cost the third 20-run round-trip attempt:
a trial sat inside a single ``spin_once`` call for 6 h 15 m while its 600 s
wall backstop was continuously overdue, because the backstop was tested on the
line after the spin (studies/engineering/roundtrip_watchdog_unreachable.json).
"""

from pathlib import Path
import threading
import time

import pytest

from modular_robot_benchmarks.spin_pump import ExecutorPump, SpinFailed


ROOT = Path(__file__).parents[1]
TRIAL = ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/mission_trial.py"


class WedgedExecutor:
    """An executor whose spin_once never returns, as rclpy's need not.

    Executor.wait_for_ready_callbacks retries internally in a while True loop,
    so a wedge burns CPU inside one spin_once call rather than raising.
    """

    def __init__(self) -> None:
        self.entered = threading.Event()

    def spin_once(self, timeout_sec=None):
        self.entered.set()
        while True:
            time.sleep(0.01)


class CountingExecutor:
    def __init__(self) -> None:
        self.calls = 0

    def spin_once(self, timeout_sec=None):
        self.calls += 1
        time.sleep(0.001)


class RaisingExecutor:
    def spin_once(self, timeout_sec=None):
        raise RuntimeError("rmw handle destroyed")


def test_the_backstop_fires_while_the_spin_is_wedged():
    executor = WedgedExecutor()
    pump = ExecutorPump(executor).start()
    assert executor.entered.wait(5.0), "the pump never entered the spin"
    start = time.monotonic()
    assert pump.wait_until(lambda: False, 0.2) is False
    elapsed = time.monotonic() - start
    assert elapsed < 5.0, f"the backstop waited on the wedged spin for {elapsed:.1f} s"


def test_a_wedged_pump_is_reported_rather_than_joined():
    executor = WedgedExecutor()
    pump = ExecutorPump(executor).start()
    assert executor.entered.wait(5.0)
    start = time.monotonic()
    assert pump.stop(timeout_s=0.2) is False
    assert time.monotonic() - start < 5.0, "stop() blocked on the wedged spin"
    assert pump.wedged is True


def test_waiting_returns_as_soon_as_the_predicate_holds():
    executor = CountingExecutor()
    pump = ExecutorPump(executor).start()
    held = []
    threading.Timer(0.05, lambda: held.append(True)).start()
    assert pump.wait_until(lambda: bool(held), 5.0) is True
    assert pump.spins > 0, "the pump served no callbacks"
    assert pump.stop() is True


def test_a_raising_spin_is_reported_not_mistaken_for_a_timeout():
    pump = ExecutorPump(RaisingExecutor()).start()
    with pytest.raises(SpinFailed, match="rmw handle destroyed"):
        pump.wait_until(lambda: False, 5.0)
    assert pump.stop() is True


def test_the_trial_never_waits_by_calling_spin_once():
    source = TRIAL.read_text()
    assert "spin_once" not in source, (
        "a wait that calls spin_once cannot reach its own deadline")
    assert "ExecutorPump(executor).start()" in source


def test_an_expired_wall_backstop_records_as_retryable_infrastructure():
    source = TRIAL.read_text()
    assert "def _watchdog_observation" in source
    assert '"infrastructure_failure"' in source
    # The simulated clock decides the mission outcome; the wall clock only
    # reports that the observer stopped making progress, so the two must not
    # share a message or a terminal status.
    assert "mission deadline exceeded on the simulated clock" in source
    assert "wall watchdog" in source


def test_each_trial_builds_its_own_context_so_a_wedge_is_survivable():
    source = TRIAL.read_text()
    assert "rclpy.Context()" in source
    assert "rclpy.init(context=context)" in source
    assert "MissionObserver(context=context)" in source
    assert "SingleThreadedExecutor(context=context)" in source
