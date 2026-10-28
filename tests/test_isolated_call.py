"""A trial's hard backstop must survive a wedge that holds the GIL.

Regression cover for the fault that stopped the 2026-10-27 round trip at r16:
the observer's spin thread blocked inside a native rclpy call that does not
release the GIL, so the in-process 600 s backstop stood 489 s overdue and
could not run (studies/engineering/roundtrip_observer_gil_wedge.json).

``ctypes.PyDLL`` keeps the GIL held across the foreign call, which reproduces
that wedge exactly; ``time.sleep``, as in test_spin_pump, releases it and
cannot.
"""

import ctypes
import os
from pathlib import Path
import threading
import time

from modular_robot_benchmarks.isolated_call import run_isolated
from modular_robot_benchmarks.spin_pump import ExecutorPump


ROOT = Path(__file__).parents[1]
BATCH = ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/mission_batch.py"
LIBC = ctypes.PyDLL(None)


def _hold_gil(seconds: int) -> None:
    LIBC.sleep(seconds)


def _hold_gil_forever() -> None:
    LIBC.pause()


def _answer() -> int:
    return 42


def _raise() -> None:
    raise ValueError("observer exploded")


def _die() -> None:
    os._exit(3)


def _answer_then_refuse_to_exit() -> int:
    # A non-daemon thread that never finishes keeps interpreter shutdown from
    # completing, as an abandoned spin can.
    threading.Thread(target=lambda: time.sleep(3600)).start()
    return 42


class GilHoldingExecutor:
    def __init__(self, seconds: int) -> None:
        self.seconds = seconds
        self.entered = threading.Event()

    def spin_once(self, timeout_sec=None):
        self.entered.set()
        _hold_gil(self.seconds)


def test_an_in_process_backstop_is_late_by_however_long_the_gil_is_held():
    # The limitation that makes process isolation necessary: the pump moves the
    # spin off the waiting thread, but the waiter still needs the GIL.
    executor = GilHoldingExecutor(3)
    pump = ExecutorPump(executor).start()
    assert executor.entered.wait(5.0)
    start = time.monotonic()
    assert pump.wait_until(lambda: False, 0.2) is False
    assert time.monotonic() - start >= 2.0
    pump.stop(timeout_s=5.0)


def test_a_child_wedged_while_holding_the_gil_is_killed_at_the_limit():
    outcome = run_isolated(_hold_gil_forever, limit_s=3.0)
    assert outcome.status == "killed"
    assert outcome.wall_s < 15.0, f"the limit was overrun by {outcome.wall_s - 3.0:.1f} s"


def test_a_result_crosses_the_process_boundary():
    outcome = run_isolated(_answer, limit_s=60.0)
    assert (outcome.status, outcome.value) == ("returned", 42)


def test_a_raising_child_reports_its_traceback():
    outcome = run_isolated(_raise, limit_s=60.0)
    assert outcome.status == "raised"
    assert "ValueError: observer exploded" in outcome.error


def test_a_child_that_dies_without_a_result_is_reported():
    outcome = run_isolated(_die, limit_s=60.0)
    assert (outcome.status, outcome.exitcode) == ("died", 3)


def test_a_delivered_result_stands_when_the_child_will_not_exit():
    outcome = run_isolated(_answer_then_refuse_to_exit, limit_s=60.0, exit_grace_s=1.0)
    assert (outcome.status, outcome.value) == ("returned", 42)
    assert outcome.wall_s < 30.0


def test_the_batch_runs_every_observer_in_its_own_process():
    source = BATCH.read_text()
    assert "observation = execute_mission_isolated(" in source
    assert "observation = execute_mission(" not in source
