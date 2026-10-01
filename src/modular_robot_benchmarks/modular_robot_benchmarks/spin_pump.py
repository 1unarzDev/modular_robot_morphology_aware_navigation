"""Keep a trial's deadlines reachable when the executor will not yield.

``rclpy.executors.Executor.wait_for_ready_callbacks`` retries internally in a
``while True`` loop, so ``spin_once`` is not guaranteed to return even with
``timeout_sec`` set. Code that waits by calling ``spin_once`` in a loop
therefore cannot reach the deadline test on the next line: during the third
20-run round-trip attempt one trial sat inside a single ``spin_once`` call for
6 h 15 m, burning 8% of a core, while its 600 s wall backstop stood
continuously overdue and the campaign made no further progress
(``studies/engineering/roundtrip_watchdog_unreachable.json``).

``ExecutorPump`` moves spinning onto its own thread. Waiting then happens on
the caller's thread, which the executor cannot wedge, so every backstop stays
reachable whether or not the spin makes progress. A pump whose spin is wedged
is reported rather than joined: the caller abandons it, and must not tear down
the context it is still spinning in.

Deadlines here are wall time, used only as a stall backstop. Nothing in this
module paces robot motion.
"""

from __future__ import annotations

import threading
import time


class SpinFailed(RuntimeError):
    """The pump's executor raised, so no further callbacks will be served."""


class ExecutorPump:
    """Spin ``executor`` on a side thread while callers wait with deadlines."""

    def __init__(self, executor, spin_timeout_s: float = 0.05,
                 poll_s: float = 0.005) -> None:
        self._executor = executor
        self._spin_timeout_s = spin_timeout_s
        self._poll_s = poll_s
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="mission-spin-pump", daemon=True)
        self._spins = 0
        self.spin_error: BaseException | None = None

    def start(self) -> "ExecutorPump":
        self._thread.start()
        return self

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._executor.spin_once(timeout_sec=self._spin_timeout_s)
            except BaseException as exc:  # surfaced to the waiter, never swallowed
                self.spin_error = exc
                return
            self._spins += 1

    @property
    def spins(self) -> int:
        """Completed spins, so a wedge can be reported with how far it got."""
        return self._spins

    @property
    def wedged(self) -> bool:
        """True once a stop was asked for and the spin did not come back."""
        return self._stop.is_set() and self._thread.is_alive()

    def wait_until(self, predicate, timeout_s: float) -> bool:
        """Wait for ``predicate`` on the caller's thread.

        Returns False once the deadline is overdue, whether or not the spin is
        making progress. A spin that raised is reported as ``SpinFailed``
        rather than left to look like a timeout.
        """
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            if predicate():
                return True
            if self.spin_error is not None:
                raise SpinFailed(
                    f"executor spin raised {type(self.spin_error).__name__}: "
                    f"{self.spin_error}") from self.spin_error
            if time.monotonic() >= deadline:
                return False
            time.sleep(self._poll_s)

    def sleep(self, duration_s: float) -> None:
        """Let the pump serve callbacks for ``duration_s`` of wall time."""
        self.wait_until(lambda: False, duration_s)

    def stop(self, timeout_s: float = 2.0) -> bool:
        """Ask the pump to stop. False if the spin is wedged and was abandoned."""
        self._stop.set()
        self._thread.join(timeout_s)
        return not self._thread.is_alive()
