from __future__ import annotations

from dataclasses import dataclass, field
import json
from math import atan2, cos, sin
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from modular_robot_msgs.action import NavigateHybrid
from modular_robot_msgs.msg import MorphologyState, RelativePoseEstimate
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
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
    reconfiguration_failures: int = 0
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
    planned_route: list[dict[str, float]] = field(default_factory=list)
    localization_history: list[dict[str, float]] = field(default_factory=list)
    pod_alignment_history: list[dict] = field(default_factory=list)
    motion_qualifications: list[dict] = field(default_factory=list)
    controller_diagnostics: dict = field(default_factory=dict)


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
        self.latest_relative_poses: dict[str, dict] = {}
        self._last_covariance_sample: float | None = None
        self._last_motion_sample: float | None = None
        self.latest_odometry: Odometry | None = None
        self.latest_odom_wall_time: float | None = None
        self.latest_scan_wall_time: float | None = None
        self.odometry_history: list[dict[str, float]] = []
        self.command_history: list[dict[str, float]] = []
        self.localization_history: list[dict[str, float]] = []
        self.pod_alignment_history: list[dict] = []
        self.sensing_revision = 0
        self.map_received = False
        self.controller_active = False
        self._controller_state_future = None
        self.latest_local_costmap: OccupancyGrid | None = None
        self.latest_collision_arc: Path | None = None
        self.topology_history: list[dict] = []
        self.execution_history: list[dict] = []
        self.pod_commands = {f"pod_{index}": 0.0 for index in range(6)}
        self.pod_signed_commands = {f"pod_{index}": 0.0 for index in range(6)}
        self.pod_angular_commands = {f"pod_{index}": 0.0 for index in range(6)}
        self.body_command = Twist()
        self.work_proxy_j = 0.0
        self.pod_drive_diagnostics: list[dict] = []
        self._pod_drive_diagnostic_time: dict[str, float] = {}
        self._previous_clock: float | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.create_subscription(MorphologyState, "morphology_state", self._on_state, qos)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap",
            self._on_local_costmap, 10)
        self.create_subscription(
            Path, "/lookahead_arc", self._on_collision_arc, 10)
        self.create_subscription(Odometry, "/odom", self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_body_command, 10)
        self.create_subscription(
            String, "/evaluator/pod_drive_diagnostics",
            self._on_pod_drive_diagnostic, qos_profile_sensor_data)
        self.create_subscription(
            RelativePoseEstimate, "relative_pose_estimate", self._on_pose, 50)
        for pod in self.pod_commands:
            self.create_subscription(
                Twist, f"/model/{pod}/cmd_vel",
                lambda message, pod=pod: self._on_command(pod, message), 10)
        self.controller_state_client = self.create_client(
            GetState, "/controller_server/get_state")
        self.create_timer(0.5, self._poll_controller_state)

    def _poll_controller_state(self) -> None:
        if (self._controller_state_future is not None
                and not self._controller_state_future.done()):
            return
        if not self.controller_state_client.service_is_ready():
            self.controller_active = False
            return
        self._controller_state_future = self.controller_state_client.call_async(
            GetState.Request())
        self._controller_state_future.add_done_callback(self._on_controller_state)

    def _on_controller_state(self, future) -> None:
        try:
            self.controller_active = (
                future.result().current_state.id == State.PRIMARY_STATE_ACTIVE)
        except Exception:
            self.controller_active = False

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
                "yaw": _yaw(pose.orientation), "angular_z": float(twist.angular.z),
            })
            self.command_history.append({
                "time_s": current,
                "body_linear_x": float(self.body_command.linear.x),
                "body_angular_z": float(self.body_command.angular.z),
                **{pod: float(value) for pod, value in self.pod_signed_commands.items()},
                **{f"{pod}_angular": float(value)
                   for pod, value in self.pod_angular_commands.items()},
            })
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "core/base_link", rclpy.time.Time())
                self.localization_history.append({
                    "time_s": current,
                    "x": float(transform.transform.translation.x),
                    "y": float(transform.transform.translation.y),
                    "yaw": _yaw(transform.transform.rotation),
                })
            except TransformException:
                pass
            self._last_motion_sample = current

    def _on_map(self, _message: OccupancyGrid) -> None:
        self.map_received = True

    def _on_local_costmap(self, message: OccupancyGrid) -> None:
        self.latest_local_costmap = message

    def _on_collision_arc(self, message: Path) -> None:
        self.latest_collision_arc = message

    def _on_odometry(self, message: Odometry) -> None:
        self.latest_odometry = message
        self.latest_odom_wall_time = time.monotonic()

    def _on_scan(self, _message: LaserScan) -> None:
        self.latest_scan_wall_time = time.monotonic()

    def _on_body_command(self, message: Twist) -> None:
        self.body_command = message

    def _on_pod_drive_diagnostic(self, message: String) -> None:
        """Retain evaluator-only plant evidence; never expose it to autonomy."""
        try:
            sample = json.loads(message.data)
        except (TypeError, ValueError):
            return
        required = {
            "time_s", "pod", "left_command_radps", "right_command_radps",
            "left_measured_radps", "right_measured_radps", "world_x",
            "world_y", "world_yaw",
        }
        if not required <= sample.keys():
            return
        pod, sample_time = str(sample["pod"]), float(sample["time_s"])
        previous = self._pod_drive_diagnostic_time.get(pod)
        if previous is not None and sample_time - previous < 0.5:
            return
        self._pod_drive_diagnostic_time[pod] = sample_time
        sample["body_linear_command_mps"] = float(self.body_command.linear.x)
        sample["body_angular_command_radps"] = float(self.body_command.angular.z)
        self.pod_drive_diagnostics.append(sample)
        # At 2 Hz per pod this covers a complete 300 s mission while preventing
        # terminal records from growing with the simulator's update rate.
        if len(self.pod_drive_diagnostics) > 3600:
            del self.pod_drive_diagnostics[:600]

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
        if (previous_topology != message.topology_revision
                or previous_execution != message.execution_state):
            # Snapshot the autonomy-side fused estimates at every observed
            # topology or execution-state change. Commit can change
            # TRANSITIONING to READY without another topology revision.
            self.pod_alignment_history.append({
                "time_s": stamp,
                "topology_revision": int(message.topology_revision),
                "morphology": message.morphology_id,
                "execution_state": execution_state_name(message.execution_state),
                "pods": {
                    pod: dict(value)
                    for pod, value in sorted(self.latest_relative_poses.items())
                },
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
        self.latest_relative_poses[message.pod_id] = {
            "x": float(message.pose.pose.position.x),
            "y": float(message.pose.pose.position.y),
            "yaw": _yaw(message.pose.pose.orientation),
            "variance_x": float(covariance[0]),
            "variance_y": float(covariance[7]),
            "variance_yaw": float(covariance[35]),
            "connector_visible": bool(message.connector_visible),
            "sources": list(message.sources),
            "sensing_revision": int(message.sensing_revision),
        }

    def _on_command(self, pod: str, message: Twist) -> None:
        self.pod_commands[pod] = abs(message.linear.x)
        self.pod_signed_commands[pod] = message.linear.x
        self.pod_angular_commands[pod] = message.angular.z

    def navigation_ready(self) -> bool:
        now = time.monotonic()
        if not (self.sim_time is not None and self.morphology is not None
                and self.map_received and self.client.server_is_ready()
                and self.controller_active
                and self.latest_odom_wall_time is not None
                and self.latest_scan_wall_time is not None
                and now - self.latest_odom_wall_time < 1.0
                and now - self.latest_scan_wall_time < 1.0):
            return False
        try:
            return self.tf_buffer.can_transform(
                "map", "core/base_link", rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            return False

    def controller_diagnostic(self) -> dict:
        """Compact final costmap/arc evidence for controller-failure diagnosis."""
        grid = self.latest_local_costmap
        odometry = self.latest_odometry
        if grid is None or odometry is None:
            return {"available": False}
        info = grid.info
        origin = info.origin
        origin_yaw = _yaw(origin.orientation)
        cos_yaw, sin_yaw = cos(origin_yaw), sin(origin_yaw)
        robot = odometry.pose.pose.position
        cells = []
        counts = {"unknown": 0, "free": 0, "inflated": 0, "inscribed_or_lethal": 0}
        for index, cost in enumerate(grid.data):
            if cost < 0:
                counts["unknown"] += 1
                continue
            if cost == 0:
                counts["free"] += 1
            elif cost >= 99:
                counts["inscribed_or_lethal"] += 1
            else:
                counts["inflated"] += 1
            if cost < 90:
                continue
            local_x = (index % info.width + 0.5) * info.resolution
            local_y = (index // info.width + 0.5) * info.resolution
            world_x = origin.position.x + cos_yaw * local_x - sin_yaw * local_y
            world_y = origin.position.y + sin_yaw * local_x + cos_yaw * local_y
            distance = ((world_x - robot.x) ** 2 + (world_y - robot.y) ** 2) ** 0.5
            if distance <= 1.5:
                cells.append({
                    "x": round(world_x, 4), "y": round(world_y, 4),
                    "cost": int(cost), "robot_distance": round(distance, 4),
                })
        cells.sort(key=lambda value: value["robot_distance"])
        arc = self.latest_collision_arc
        return {
            "available": True,
            "frame_id": grid.header.frame_id,
            "resolution": float(info.resolution),
            "width": int(info.width), "height": int(info.height),
            "cost_counts": counts,
            "high_cost_cells_within_1_5m": cells[:512],
            "high_cost_cells_truncated": len(cells) > 512,
            "robot_pose": {
                "x": float(robot.x), "y": float(robot.y),
                "yaw": _yaw(odometry.pose.pose.orientation),
            },
            "collision_arc": ([{
                "x": float(pose.pose.position.x), "y": float(pose.pose.position.y),
                "yaw": _yaw(pose.pose.orientation),
            } for pose in arc.poses] if arc else []),
            "pod_drive_diagnostics_source": "evaluator_only_gazebo",
            "pod_drive_diagnostic_samples": list(self.pod_drive_diagnostics),
        }


def execution_state_name(value: int) -> str:
    return {
        MorphologyState.READY: "READY",
        MorphologyState.TRANSITIONING: "TRANSITIONING",
        MorphologyState.RECOVERY_REQUIRED: "RECOVERY_REQUIRED",
        MorphologyState.STOPPED: "STOPPED",
    }.get(value, f"UNKNOWN_{value}")


def _yaw(quaternion) -> float:
    return atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


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
            return _observation(node, "infrastructure_failure", False,
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
        terminal_status=status, completed=completed, message=message,
        simulated_duration_s=simulated,
        wall_duration_s=time.monotonic() - wall_start,
        reconfiguration_attempts=attempts,
        reconfiguration_failures=(
            1 if final_state == "RECOVERY_REQUIRED" and attempts else 0),
        mechanical_work_j=node.work_proxy_j,
        covariance_trace=node.covariance_trace,
        topology_history=node.topology_history,
        execution_state_history=node.execution_history,
        final_execution_state=final_state,
        unrecovered_fault=final_state != "READY",
        topology_revision=int(state.topology_revision) if state else 0,
        sensing_revision=node.sensing_revision,
        planning_latency_s=(
            float(navigation_result.planning_latency) if navigation_result else 0.0),
        expanded_states=(int(navigation_result.expanded_states) if navigation_result else 0),
        map_revision=(int(navigation_result.map_revision) if navigation_result else 0),
        planned_route_signature=(
            navigation_result.planned_route_signature if navigation_result else ""),
        planned_transition_sites=([{"transition_id": transition_id,
           "x": pose.pose.position.x, "y": pose.pose.position.y}
          for transition_id, pose in zip(
              navigation_result.planned_transition_ids,
              navigation_result.planned_transition_poses)]
         if navigation_result else []),
        odometry_history=node.odometry_history,
        command_history=node.command_history,
        planned_route=([{"x": pose.pose.position.x, "y": pose.pose.position.y,
           "orientation_z": pose.pose.orientation.z,
           "orientation_w": pose.pose.orientation.w}
          for pose in navigation_result.planned_route_poses]
         if navigation_result else []),
        localization_history=node.localization_history,
        pod_alignment_history=node.pod_alignment_history,
        motion_qualifications=(
            [json.loads(value) for value in navigation_result.motion_qualification_json]
            if navigation_result else []),
        controller_diagnostics=node.controller_diagnostic(),
    )
