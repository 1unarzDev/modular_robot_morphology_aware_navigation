from __future__ import annotations

from dataclasses import dataclass, field
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from modular_robot_msgs.action import NavigateHybrid
from modular_robot_msgs.msg import MorphologyState, RelativePoseEstimate
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformException, TransformListener

from .mission_batch import classify_terminal


@dataclass
class MissionObservation:
    terminal_status: str
    completed: bool
    message: str
    simulated_duration_s: float
    wall_duration_s: float
    reconfiguration_attempts: int = 0
    mechanical_work_j: float = 0.0
    covariance_trace: list[float] = field(default_factory=list)
    topology_history: list[dict] = field(default_factory=list)
    execution_state_history: list[dict] = field(default_factory=list)
    final_execution_state: str = "STOPPED"
    unrecovered_fault: bool = False
    topology_revision: int = 0
    sensing_revision: int = 0
    planning_latency_s: float = 0.0
    expanded_states: int = 0
    map_revision: int = 0
    planned_route_signature: str = ""
    planned_transition_sites: list[dict] = field(default_factory=list)
    odometry_history: list[dict[str, float]] = field(default_factory=list)
    command_history: list[dict[str, float]] = field(default_factory=list)


class MissionObserver(Node):
    def __init__(self) -> None:
        super().__init__("morphology_mission_observer")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.client = ActionClient(self, NavigateHybrid, "navigate_hybrid")
        self.sim_time: float | None = None
        self.morphology: MorphologyState | None = None
        self.covariance_trace: list[float] = []
        self.latest_covariance: dict[str, float] = {}
        self._last_covariance_sample: float | None = None
        self._last_motion_sample: float | None = None
        self.latest_odometry: Odometry | None = None
        self.odometry_history: list[dict[str, float]] = []
        self.command_history: list[dict[str, float]] = []
        self.sensing_revision = 0
        self.map_received = False
        self.topology_history: list[dict] = []
        self.execution_history: list[dict] = []
        self.pod_commands = {f"pod_{index}": 0.0 for index in range(6)}
        self.work_proxy_j = 0.0
        self._previous_clock: float | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.create_subscription(MorphologyState, "morphology_state", self._on_state, qos)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)
        self.create_subscription(Odometry, "/odom", self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(
            RelativePoseEstimate, "relative_pose_estimate", self._on_pose, 50)
        for pod in self.pod_commands:
            self.create_subscription(
                Twist, f"/model/{pod}/cmd_vel",
                lambda message, pod=pod: self._on_command(pod, message), 10)

    def _on_clock(self, message: Clock) -> None:
        current = message.clock.sec + message.clock.nanosec * 1e-9
        if self._previous_clock is not None and current >= self._previous_clock:
            # Nominal 10 N rolling-force proxy per pod. The manifest labels this
            # explicitly; it is not battery or motor-electrical energy.
            self.work_proxy_j += 10.0 * sum(self.pod_commands.values()) * (
                current - self._previous_clock)
        self._previous_clock = current
        self.sim_time = current
        if (self.latest_covariance and
                (self._last_covariance_sample is None
                 or current - self._last_covariance_sample >= 0.5)):
            self.covariance_trace.append(
                sum(self.latest_covariance.values()) / len(self.latest_covariance))
            self._last_covariance_sample = current
        if (self.latest_odometry is not None and
                (self._last_motion_sample is None
                 or current - self._last_motion_sample >= 0.5)):
            pose = self.latest_odometry.pose.pose
            twist = self.latest_odometry.twist.twist
            self.odometry_history.append({
                "time_s": current, "x": float(pose.position.x),
                "y": float(pose.position.y), "linear_x": float(twist.linear.x),
                "angular_z": float(twist.angular.z),
            })
            self.command_history.append({
                "time_s": current,
                **{pod: float(value) for pod, value in self.pod_commands.items()},
            })
            self._last_motion_sample = current

    def _on_map(self, _message: OccupancyGrid) -> None:
        self.map_received = True

    def _on_odometry(self, message: Odometry) -> None:
        self.latest_odometry = message

    def _on_state(self, message: MorphologyState) -> None:
        previous_topology = self.morphology.topology_revision if self.morphology else None
        previous_execution = self.morphology.execution_state if self.morphology else None
        self.morphology = message
        stamp = self.sim_time or 0.0
        if previous_topology != message.topology_revision:
            self.topology_history.append({
                "time_s": stamp, "revision": int(message.topology_revision),
                "morphology": message.morphology_id,
                "graph_hash": message.topology.graph_hash,
            })
        if previous_execution != message.execution_state:
            self.execution_history.append({
                "time_s": stamp, "state": execution_state_name(message.execution_state),
            })

    def _on_pose(self, message: RelativePoseEstimate) -> None:
        covariance = message.pose.covariance
        self.latest_covariance[message.pod_id] = float(
            covariance[0] + covariance[7] + covariance[35])
        self.sensing_revision = max(self.sensing_revision, int(message.sensing_revision))

    def _on_command(self, pod: str, message: Twist) -> None:
        self.pod_commands[pod] = abs(message.linear.x)

    def navigation_ready(self) -> bool:
        if not (self.sim_time is not None and self.morphology is not None
                and self.map_received and self.client.server_is_ready()):
            return False
        try:
            return self.tf_buffer.can_transform(
                "map", "core/base_link", rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            return False


def execution_state_name(value: int) -> str:
    return {
        MorphologyState.READY: "READY",
        MorphologyState.TRANSITIONING: "TRANSITIONING",
        MorphologyState.RECOVERY_REQUIRED: "RECOVERY_REQUIRED",
        MorphologyState.STOPPED: "STOPPED",
    }.get(value, f"UNKNOWN_{value}")


def execute_mission(
    goal_xy: tuple[float, float], method: str, deadline_s: float,
    wall_watchdog_s: float,
) -> MissionObservation:
    rclpy.init()
    node = MissionObserver()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    wall_start = time.monotonic()
    sim_start = None
    timed_out = False
    message = ""
    success = False
    attempts = 0
    try:
        while time.monotonic() - wall_start < min(60.0, wall_watchdog_s):
            executor.spin_once(timeout_sec=0.1)
            if node.navigation_ready():
                break
        else:
            return _observation(node, "stale_topic", False,
                                "stack readiness timeout", 0.0, wall_start, 0)
        sim_start = node.sim_time
        goal = NavigateHybrid.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = "map"
        goal.goal.pose.position.x, goal.goal.pose.position.y = goal_xy
        goal.goal.pose.orientation.w = 1.0
        goal.planner_method = method
        sent = node.client.send_goal_async(goal)
        while not sent.done() and time.monotonic() - wall_start < wall_watchdog_s:
            executor.spin_once(timeout_sec=0.1)
        handle = sent.result() if sent.done() else None
        if handle is None or not handle.accepted:
            return _observation(node, "controller_failure", False,
                                "NavigateHybrid goal rejected", 0.0, wall_start, 0)
        result_future = handle.get_result_async()
        while not result_future.done():
            executor.spin_once(timeout_sec=0.1)
            simulated = (node.sim_time or sim_start) - sim_start
            if simulated >= deadline_s or time.monotonic() - wall_start >= wall_watchdog_s:
                timed_out = True
                handle.cancel_goal_async()
                break
        if not timed_out:
            wrapped = result_future.result()
            success = bool(wrapped.result.success)
            message = wrapped.result.message
            attempts = int(wrapped.result.reconfiguration_count)
            navigation_result = wrapped.result
        else:
            navigation_result = None
            message = "mission deadline or wall watchdog exceeded"
        simulated = max(0.0, (node.sim_time or sim_start) - sim_start)
        return _observation(
            node, classify_terminal(success, message, timed_out), success,
            message, simulated, wall_start, attempts, navigation_result)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _observation(node, status, completed, message, simulated, wall_start, attempts,
                 navigation_result=None):
    state = node.morphology
    final_state = execution_state_name(state.execution_state) if state else "STOPPED"
    safe_completion = completed and final_state == "READY"
    if completed and not safe_completion:
        status, completed = "unsafe_topology", False
    return MissionObservation(
        status, completed, message, simulated, time.monotonic() - wall_start,
        attempts, node.work_proxy_j, node.covariance_trace,
        node.topology_history, node.execution_history, final_state,
        final_state != "READY", int(state.topology_revision) if state else 0,
        node.sensing_revision,
        float(navigation_result.planning_latency) if navigation_result else 0.0,
        int(navigation_result.expanded_states) if navigation_result else 0,
        int(navigation_result.map_revision) if navigation_result else 0,
        navigation_result.planned_route_signature if navigation_result else "",
        ([{"transition_id": transition_id,
           "x": pose.pose.position.x, "y": pose.pose.position.y}
          for transition_id, pose in zip(
              navigation_result.planned_transition_ids,
              navigation_result.planned_transition_poses)]
         if navigation_result else []),
        node.odometry_history,
        node.command_history,
    )
