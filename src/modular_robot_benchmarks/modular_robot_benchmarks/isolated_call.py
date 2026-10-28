"""Run one call in its own process, under a wall limit nothing in it can defeat.

``spin_pump.ExecutorPump`` keeps a trial's backstops reachable while the
executor is wedged in Python. It cannot when the wedge holds the GIL. In the
2026-10-27 round trip the pump's thread entered ``rcl_action_client_fini`` from
inside ``EventHandler.is_ready`` -- a native call that does not release the GIL
-- and blocked on a Fast-DDS writer mutex. The waiting thread then could not
reacquire the GIL to test its deadline, which stood 489 s overdue while the
trial's simulation ran on past 530 s of a 300 s deadline
(``studies/engineering/roundtrip_observer_gil_wedge.json``). Any backstop that
needs the GIL shares that fate, and every thread in the process needs it.

A separate process does not. The parent holds no ROS participant at all, waits
on a pipe, and kills the child once the limit is overdue, whatever the child is
blocked in. A fresh process per trial also leaves no participant to carry state
from one trial's stack into the next.

Deadlines here are wall time, used only as a stall backstop. Nothing in this
module paces robot motion.
"""

from __future__ import annotations

from dataclasses import dataclass
import multiprocessing
import time
import traceback
from typing import Any, Callable


@dataclass
class IsolatedOutcome:
    """How the isolated call ended.

    ``status`` is ``returned`` (``value`` holds the result), ``raised``
    (``error`` holds the child's traceback), ``killed`` (the limit expired and
    the child was killed) or ``died`` (the child exited without a result).
    """

    status: str
    value: Any = None
    error: str = ""
    exitcode: int | None = None
    wall_s: float = 0.0


def _child(sender, target: Callable, args: tuple, kwargs: dict) -> None:
    try:
        result = ("returned", target(*args, **kwargs))
    except BaseException:  # reported to the parent, never swallowed
        result = ("raised", traceback.format_exc())
    sender.send(result)
    sender.close()


def run_isolated(target: Callable, args: tuple = (), kwargs: dict | None = None,
                 limit_s: float = 60.0, exit_grace_s: float = 10.0) -> IsolatedOutcome:
    """Call ``target(*args, **kwargs)`` in a spawned child, killing it after ``limit_s``.

    ``target`` and its arguments must be importable and picklable. A child that
    has delivered its result but does not exit within ``exit_grace_s`` -- for
    example because an abandoned spin keeps interpreter shutdown from
    finishing -- is killed too; the result it sent stands.
    """
    context = multiprocessing.get_context("spawn")
    receiver, sender = context.Pipe(duplex=False)
    process = context.Process(
        target=_child, args=(sender, target, args, kwargs or {}),
        name="isolated-call", daemon=True)
    start = time.monotonic()
    process.start()
    # Only the child may hold the sending end, or EOF never arrives on death.
    sender.close()
    message = None
    try:
        if receiver.poll(max(0.0, limit_s)):
            message = receiver.recv()
    except EOFError:
        message = None
    finally:
        receiver.close()
    if message is None:
        timed_out = process.is_alive()
        process.join(0 if timed_out else exit_grace_s)
    else:
        process.join(exit_grace_s)
    if process.is_alive():
        process.kill()
        process.join()
    wall_s = time.monotonic() - start
    if message is not None:
        status, payload = message
        if status == "returned":
            return IsolatedOutcome("returned", value=payload,
                                   exitcode=process.exitcode, wall_s=wall_s)
        return IsolatedOutcome("raised", error=payload,
                               exitcode=process.exitcode, wall_s=wall_s)
    if timed_out:
        return IsolatedOutcome("killed", exitcode=process.exitcode, wall_s=wall_s)
    return IsolatedOutcome("died", exitcode=process.exitcode, wall_s=wall_s)
