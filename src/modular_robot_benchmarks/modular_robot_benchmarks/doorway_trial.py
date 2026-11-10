"""One ExperimentChoice doorway mission through stock Nav2 (EXPLORATORY).

The same observer, readiness gates, simulated-clock deadline and wall backstop
as `mission_trial.execute_mission`; the only difference is who is asked to
drive. The goal goes to bt_navigator's `navigate_to_pose` (planner server,
controller server and the default behaviour tree with its recoveries), not to
the hybrid navigator, which stays idle.

NOT YET RUN: written against the Nav2 Jazzy action definitions without a ROS
installation at hand. The canary is this module's first execution.
"""
from __future__ import annotations

import json
from math import cos, sin
import time

import rclpy
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from .doorway_outcome import footprint_edges, nav2_terminal
from .isolated_call import run_isolated
from .mission_trial import (
    OBSERVER_PROCESS_GRACE_S, MissionObservation, MissionObserver, _observation,
    _remaining, _watchdog_observation,
)
from .spin_pump import ExecutorPump


def execute_doorway_mission(
    goal_xy: tuple[float, float], goal_yaw: float, deadline_s: float,
    wall_watchdog_s: float,
) -> MissionObservation:
    context = rclpy.Context()
    rclpy.init(context=context)
    node = MissionObserver(context=context)
    navigate = ActionClient(node, NavigateToPose, "navigate_to_pose")
    recoveries = {"max": 0}
    # The rectangle Nav2 actually collision-checks, as the costmap publishes it.
    footprint = {"points": []}
    node.create_subscription(
        PolygonStamped, "/local_costmap/published_footprint",
        lambda message: footprint.__setitem__(
            "points", [(float(point.x), float(point.y)) for point in message.polygon.points]),
        10)

    def on_feedback(message) -> None:
        recoveries["max"] = max(
            recoveries["max"], int(getattr(message.feedback, "number_of_recoveries", 0)))

    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    pump = ExecutorPump(executor).start()
    wall_start = time.monotonic()
    try:
        def ready() -> bool:
            return node.navigation_ready() and navigate.server_is_ready()

        if not pump.wait_until(ready, min(60.0, wall_watchdog_s)):
            readiness = {**node.navigation_readiness(),
                         "navigate_to_pose_action": navigate.server_is_ready()}
            return _observation(node, "infrastructure_failure", False,
                                "stack readiness timeout: "
                                + json.dumps(readiness, sort_keys=True),
                                0.0, wall_start, 0)
        node.ready_wall_time = time.monotonic()
        sim_start = node.sim_time
        node.mission_started = True
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = goal_xy
        goal.pose.pose.orientation.z = sin(goal_yaw / 2.0)
        goal.pose.pose.orientation.w = cos(goal_yaw / 2.0)
        sent = navigate.send_goal_async(goal, feedback_callback=on_feedback)
        if not pump.wait_until(sent.done, _remaining(wall_start, wall_watchdog_s)):
            return _watchdog_observation(node, pump, wall_watchdog_s, wall_start,
                                         0.0, "before the goal was accepted")
        handle = sent.result()
        if handle is None or not handle.accepted:
            return _observation(node, "controller_failure", False,
                                "NavigateToPose goal rejected", 0.0, wall_start, 0)
        result_future = handle.get_result_async()

        def settled() -> bool:
            return (result_future.done()
                    or (node.sim_time or sim_start) - sim_start >= deadline_s)

        reached = pump.wait_until(settled, _remaining(wall_start, wall_watchdog_s))
        timed_out = False
        goal_status, error_code, error_message = 0, 0, ""
        if not result_future.done():
            handle.cancel_goal_async()
            if not reached:
                return _watchdog_observation(
                    node, pump, wall_watchdog_s, wall_start,
                    max(0.0, (node.sim_time or sim_start) - sim_start),
                    "with the mission unfinished")
            timed_out = True
        else:
            wrapped = result_future.result()
            goal_status = int(wrapped.status)
            error_code = int(getattr(wrapped.result, "error_code", 0))
            error_message = str(getattr(wrapped.result, "error_msg", ""))
        status, completed = nav2_terminal(goal_status, error_code, timed_out)
        message = ("mission deadline exceeded on the simulated clock" if timed_out else
                   f"nav2 navigate_to_pose goal status {goal_status}, error code "
                   f"{error_code}" + (f": {error_message}" if error_message else ""))
        simulated = max(0.0, (node.sim_time or sim_start) - sim_start)
        pump.stop()
        observation = _observation(
            node, status, completed, message, simulated, wall_start, 0)
        observation.recovery_actions = recoveries["max"]
        observation.controller_diagnostics["nav2_result"] = {
            "goal_status": goal_status, "error_code": error_code,
            "error_msg": error_message, "number_of_recoveries": recoveries["max"],
            "timed_out": timed_out,
            "local_costmap_footprint_edges_m": footprint_edges(footprint["points"])}
        return observation
    finally:
        if pump.stop():
            executor.remove_node(node)
            node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)


def execute_doorway_mission_isolated(
    goal_xy: tuple[float, float], goal_yaw: float, deadline_s: float,
    wall_watchdog_s: float,
) -> MissionObservation:
    """`execute_doorway_mission` in its own process; see `execute_mission_isolated`."""
    limit_s = wall_watchdog_s + OBSERVER_PROCESS_GRACE_S
    outcome = run_isolated(
        execute_doorway_mission, (goal_xy, goal_yaw, deadline_s, wall_watchdog_s),
        limit_s=limit_s)
    if outcome.status == "returned":
        return outcome.value
    if outcome.status == "raised":
        raise RuntimeError("observer process raised: " + outcome.error.strip())
    if outcome.status == "killed":
        detail = (f"observer process exceeded the hard wall limit "
                  f"{limit_s:.0f} s and was killed")
    else:
        detail = f"observer process exited {outcome.exitcode} without a result"
    return MissionObservation("infrastructure_failure", False, detail, 0.0, outcome.wall_s)
