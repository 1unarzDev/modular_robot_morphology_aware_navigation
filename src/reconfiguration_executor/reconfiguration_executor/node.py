from __future__ import annotations

import asyncio
from math import atan2, hypot, pi
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from modular_robot_msgs.action import ExecuteReconfiguration
from modular_robot_msgs.msg import ConnectorCommand, ConnectorState, RelativePoseEstimate as RelativePoseEstimateMsg
from modular_robot_msgs.srv import BeginTransition, FailTransition, ObserveConnector, SetLocomotionMode
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .control import Pose2, VelocityCommand, docking_command, injected_failure, wrap_angle
from .sensing import DockingEvidence, RelativePoseEstimate, UncertaintyClass, docking_acceptance


class ReconfigurationExecutor(Node):
    """Moves one detached pod at a time and commits only verified topology."""

    def __init__(self) -> None:
        super().__init__("reconfiguration_executor")
        default_catalog = (
            get_package_share_directory("modular_robot_description")
            + "/config/morphologies.yaml"
        )
        self.declare_parameter("catalog", default_catalog)
        self.declare_parameter("state_timeout", 3.0)
        self.declare_parameter("relocation_timeout", 45.0)
        self.declare_parameter("control_rate", 30.0)
        self.declare_parameter("position_tolerance", 0.012)
        # Match the declared connector capture envelope (3 degrees). The
        # previous 0.08 rad gate admitted visibly misaligned rigid arrays.
        self.declare_parameter("yaw_tolerance", 0.05236)
        self.declare_parameter("relocation_position_tolerance", 0.006)
        # The long 0.20 m track amplifies residual pod yaw into translation
        # during body rotation. Require sub-degree alignment before latching.
        self.declare_parameter("relocation_yaw_tolerance", 0.012)
        self.declare_parameter("max_pod_linear", 0.22)
        self.declare_parameter("max_pod_angular", 0.9)
        # The bounded effort actuator must exceed wheel/ground static friction.
        # Lower nonzero yaw commands can leave both wheels stalled indefinitely.
        self.declare_parameter("min_pod_angular", 0.60)
        self.declare_parameter("post_latch_settle_timeout", 1.5)
        self.declare_parameter("docking_stable_samples", 3)
        self.declare_parameter("failure_injection", "")
        self.declare_parameter("require_connector_visibility", True)
        with open(self.get_parameter("catalog").value, encoding="utf-8") as stream:
            self.catalog = yaml.safe_load(stream)

        self.group = ReentrantCallbackGroup()
        self.poses: dict[str, Pose2] = {}
        self.pose_times: dict[str, float] = {}
        self.estimates: dict[str, RelativePoseEstimate] = {}
        self.relative_speeds: dict[str, tuple[float, float]] = {}
        self.joint_events: dict[str, tuple[str, int]] = {}
        self.event_counter = 0
        self.command_pub = self.create_publisher(ConnectorCommand, "connector_command", 10)
        self.state_pub = self.create_publisher(ConnectorState, "connector_state", 20)
        self.transport_pub = self.create_publisher(String, "topology_joint_command", 10)
        self.create_subscription(
            String, "topology_joint_state", self._on_joint_state, 20,
            callback_group=self.group,
        )
        self.create_subscription(
            RelativePoseEstimateMsg, "relative_pose_estimate", self._on_relative_pose, 50,
            callback_group=self.group,
        )
        self.pod_publishers = {
            pod: self.create_publisher(Twist, f"/model/{pod}/cmd_vel", 10)
            for pod, value in self.catalog["inventory"].items()
            if value["type"] == "steer_drive_pod"
        }
        self.mode_client = self.create_client(
            SetLocomotionMode, "set_locomotion_mode", callback_group=self.group
        )
        self.begin_client = self.create_client(
            BeginTransition, "begin_transition", callback_group=self.group
        )
        self.fail_client = self.create_client(
            FailTransition, "fail_transition", callback_group=self.group
        )
        self.observe_client = self.create_client(
            ObserveConnector, "observe_connector", callback_group=self.group
        )
        self.drive_client = self.create_client(
            SetBool, "set_assembly_drive_enabled", callback_group=self.group
        )
        self.server = ActionServer(
            self, ExecuteReconfiguration, "execute_reconfiguration", self._execute,
            callback_group=self.group,
        )

    def _on_joint_state(self, message: String) -> None:
        parts = message.data.split("|")
        if len(parts) != 2 or parts[0] not in {"attached", "detached"}:
            return
        pod = parts[1].split("::", 1)[0]
        self.event_counter += 1
        self.joint_events[pod] = (parts[0], self.event_counter)

    def _on_relative_pose(self, message: RelativePoseEstimateMsg) -> None:
        quaternion = message.pose.pose.orientation
        yaw = 2.0 * atan2(quaternion.z, quaternion.w)
        pose = Pose2(message.pose.pose.position.x, message.pose.pose.position.y, yaw)
        received = time.monotonic()
        previous, previous_time = self.poses.get(message.pod_id), self.pose_times.get(message.pod_id)
        if previous is not None and previous_time is not None and received > previous_time:
            dt = received - previous_time
            linear = ((pose.x - previous.x) ** 2 + (pose.y - previous.y) ** 2) ** 0.5 / dt
            angular = abs(pose.yaw - previous.yaw) / dt
            self.relative_speeds[message.pod_id] = (linear, angular)
        self.poses[message.pod_id] = pose
        self.pose_times[message.pod_id] = received
        self.estimates[message.pod_id] = RelativePoseEstimate(
            message.pod_id, received, pose, message.pose.covariance[0],
            message.pose.covariance[7], message.pose.covariance[35],
            UncertaintyClass(message.uncertainty_class), message.connector_visible,
            tuple(message.sources), message.sensing_revision,
        )

    async def _execute(self, goal_handle):
        transition = next(
            (t for t in self.catalog["transitions"] if t["id"] == goal_handle.request.transition_id),
            None,
        )
        result = ExecuteReconfiguration.Result()
        if transition is None:
            result.message = "unknown transition"
            goal_handle.abort()
            return result
        begin = await self._begin(transition, goal_handle.request)
        if begin is None or not begin.success:
            result.message = begin.message if begin else "morphology manager unavailable"
            goal_handle.abort()
            return result
        if not await self._set_drive_enabled(False):
            await self._report_failure(transition["id"], "assembly drive adapter unavailable")
            result.message = "assembly drive adapter unavailable"
            goal_handle.abort()
            return result

        started = time.monotonic()
        active_pod = ""
        phase = "begin"
        try:
            pods = transition.get("moved_pods", [])
            for index, pod in enumerate(pods):
                active_pod = pod
                if goal_handle.is_cancel_requested or self._inject("cancellation", pod):
                    raise asyncio.CancelledError
                phase = "unlatch"
                self._feedback(goal_handle, phase, pod, index, len(pods))
                if self._inject("stale_feedback", pod):
                    raise RuntimeError(f"injected stale feedback for {pod}")
                if self._inject("detach", pod):
                    raise RuntimeError(f"injected detach failure for {pod}")
                since = self.event_counter
                await self._command_joint(pod, ConnectorCommand.DETACH, transition["from"])
                if not await self._wait_for_joint(pod, "detached", since):
                    raise RuntimeError(f"{pod} detach was not confirmed")
                if not await self._record_connector(pod, transition["from"], False, "detach confirmed"):
                    raise RuntimeError(f"manager rejected {pod} detach observation")

                phase = "relocate"
                self._feedback(goal_handle, phase, pod, index, len(pods))
                if self._inject("relocation", pod):
                    raise RuntimeError(f"injected relocation failure for {pod}")
                target = self.catalog["morphologies"][transition["to"]]["pods"][pod]
                waypoints = [*transition.get("pod_waypoints", {}).get(pod, [])]
                if not waypoints or waypoints[-1] != target:
                    waypoints.append(target)
                for waypoint_index, waypoint in enumerate(waypoints):
                    is_final_target = waypoint_index == len(waypoints) - 1
                    if not await self._move_pod(
                            pod, waypoint, goal_handle, precise=is_final_target):
                        raise RuntimeError(
                            f"{pod} did not reach relocation waypoint {waypoint_index + 1}/{len(waypoints)}"
                        )
                accepted, reasons = self._docking_ready(pod, target, latch_confirmed=False)
                # Latch confirmation occurs after the command, so exclude only
                # that one expected pre-latch rejection.
                pre_latch_reasons = tuple(reason for reason in reasons if reason != "latch_unconfirmed")
                if pre_latch_reasons:
                    raise RuntimeError(f"{pod} docking evidence rejected: {','.join(pre_latch_reasons)}")

                phase = "latch"
                self._feedback(goal_handle, phase, pod, index, len(pods))
                if self._inject("latch", pod):
                    raise RuntimeError(f"injected latch failure for {pod}")
                since = self.event_counter
                await self._command_joint(
                    pod, ConnectorCommand.ATTACH, transition["to"], target)
                if not await self._wait_for_joint(pod, "attached", since):
                    raise RuntimeError(f"{pod} attach was not confirmed")
                # Notify the estimator of the discrete connector seating
                # transform before validating post-latch observations. The
                # manager topology is committed only after validation below.
                self._publish_connector_state(
                    pod, transition["to"], True,
                    "physical latch confirmed; validation pending")
                accepted, reasons = await self._wait_for_docking_ready(
                    pod, target, latch_confirmed=True,
                    timeout=float(self.get_parameter("post_latch_settle_timeout").value),
                )
                if not accepted:
                    raise RuntimeError(f"{pod} post-latch evidence rejected: {','.join(reasons)}")
                if not await self._record_connector(pod, transition["to"], True, "latch confirmed"):
                    raise RuntimeError(f"manager rejected {pod} latch observation")

            phase, active_pod = "commit", ""
            self._feedback(goal_handle, phase, active_pod, len(pods), len(pods))
            if self._inject("manager_commit"):
                raise RuntimeError("injected manager commit failure")
            response = await self._commit(
                transition,
                goal_handle.request.expected_topology_revision + 2 * len(pods),
            )
            if response is None or not response.success:
                message = response.message if response else "morphology manager unavailable"
                raise RuntimeError(message)
        except asyncio.CancelledError:
            await self._report_failure(transition["id"], "cancelled")
            result.message = "transition cancelled; topology not committed"
            goal_handle.canceled()
            return result
        except RuntimeError as exc:
            detail = self._failure_detail(active_pod, phase, str(exc))
            self.get_logger().error(detail)
            await self._report_failure(transition["id"], detail)
            result.message = f"transition failed; topology not committed: {detail}"
            goal_handle.abort()
            return result
        finally:
            self._stop_all()

        if not await self._set_drive_enabled(True):
            result.message = "transition committed but drive adapter could not be enabled"
            goal_handle.abort()
            return result
        result.success = True
        result.message = "transition completed with observed latch topology verified"
        result.resulting_morphology = transition["to"]
        result.observed_time = time.monotonic() - started
        result.observed_energy = float(transition["energy"])
        goal_handle.succeed()
        return result

    def _failure_detail(self, pod: str, phase: str, reason: str) -> str:
        fields = [f"phase={phase}", f"pod={pod or 'none'}", f"reason={reason}"]
        estimate = self.estimates.get(pod)
        if estimate is not None:
            fields.extend([
                f"relative_pose=({estimate.pose.x:.4f},{estimate.pose.y:.4f},{estimate.pose.yaw:.4f})",
                f"covariance=({estimate.variance_x:.6g},{estimate.variance_y:.6g},{estimate.variance_yaw:.6g})",
                f"connector_visible={str(estimate.visible).lower()}",
                f"sources={','.join(estimate.sources)}",
            ])
        joint = self.joint_events.get(pod)
        fields.append(f"latch_observation={joint[0] if joint else 'missing'}")
        return "; ".join(fields)

    async def _set_drive_enabled(self, enabled: bool) -> bool:
        if not self.drive_client.wait_for_service(timeout_sec=2.0):
            return False
        request = SetBool.Request()
        request.data = enabled
        response = await self.drive_client.call_async(request)
        return bool(response.success)

    def _inject(self, stage: str, pod: str = "") -> bool:
        return injected_failure(
            str(self.get_parameter("failure_injection").value), stage, pod
        )

    async def _command_joint(
        self, pod: str, operation: int, morphology: str,
        target: list[float] | None = None,
    ) -> None:
        command = ConnectorCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.operation = operation
        command.pod_id = pod
        command.parent_port = f"{morphology}/{pod}"
        self.command_pub.publish(command)
        transport = String()
        name = "detach" if operation == ConnectorCommand.DETACH else "attach"
        transport.data = f"{name}|{pod}"
        if operation == ConnectorCommand.ATTACH and target is not None:
            transport.data += "|" + "|".join(
                f"{float(value):.12g}" for value in target)
        self.transport_pub.publish(transport)
        await self._sleep(0.02)

    async def _wait_for_joint(self, pod: str, expected: str, since: int) -> bool:
        deadline = time.monotonic() + float(self.get_parameter("state_timeout").value)
        while time.monotonic() < deadline:
            state = self.joint_events.get(pod)
            if state and state[0] == expected and state[1] > since:
                return True
            await self._sleep(0.02)
        return False

    async def _wait_for_pose(self, module: str) -> bool:
        deadline = time.monotonic() + float(self.get_parameter("state_timeout").value)
        while time.monotonic() < deadline:
            if module in self.poses and time.monotonic() - self.pose_times[module] < 0.5:
                return True
            await self._sleep(0.02)
        return False

    async def _move_pod(
        self, pod: str, relative_target: list[float], goal_handle,
        precise: bool = True,
    ) -> bool:
        if not await self._wait_for_pose(pod):
            return False
        deadline = time.monotonic() + float(self.get_parameter("relocation_timeout").value)
        period = 1.0 / float(self.get_parameter("control_rate").value)
        position_tolerance = (
            float(self.get_parameter("relocation_position_tolerance").value)
            if precise else 0.03)
        yaw_tolerance = (
            float(self.get_parameter("relocation_yaw_tolerance").value)
            if precise else 0.20)
        final_alignment = False
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                raise asyncio.CancelledError
            current = self.poses.get(pod)
            if current is None:
                await self._sleep(period)
                continue
            target = Pose2(*(float(value) for value in relative_target))
            distance = hypot(target.x - current.x, target.y - current.y)
            if distance <= max(0.05, 3.0 * position_tolerance):
                final_alignment = True
            elif final_alignment and distance > 0.08:
                final_alignment = False
            if final_alignment:
                yaw_error = wrap_angle(target.yaw - current.yaw)
                if distance > position_tolerance:
                    # A differential pod cannot remove lateral error while it
                    # holds the terminal connector yaw.  Close position with
                    # the normal forward/reverse docking law first, then align
                    # the connector once inside the translation tolerance.
                    command, arrived = docking_command(
                        current, target, position_tolerance, pi,
                        min(0.08, float(
                            self.get_parameter("max_pod_linear").value)),
                        float(self.get_parameter("max_pod_angular").value),
                    )
                elif abs(yaw_error) > yaw_tolerance:
                    minimum = float(
                        self.get_parameter("min_pod_angular").value)
                    maximum = float(
                        self.get_parameter("max_pod_angular").value)
                    magnitude = min(maximum, max(minimum, 1.5 * abs(yaw_error)))
                    command = VelocityCommand(
                        0.0, magnitude if yaw_error > 0.0 else -magnitude)
                    arrived = False
                else:
                    command, arrived = VelocityCommand(), True
            else:
                command, arrived = docking_command(
                    current, target, position_tolerance, yaw_tolerance,
                    float(self.get_parameter("max_pod_linear").value),
                    float(self.get_parameter("max_pod_angular").value),
                )
            minimum_angular = float(
                self.get_parameter("min_pod_angular").value)
            # Apply breakaway only for an in-place turn.  During the final
            # translational correction both wheels already receive drive
            # effort, and forcing this angular floor causes limit cycling at
            # millimetre-scale latch tolerances.
            if (abs(command.linear) < 1e-6
                    and 0.0 < abs(command.angular) < minimum_angular):
                command = VelocityCommand(
                    command.linear,
                    minimum_angular if command.angular > 0.0 else -minimum_angular)
            output = Twist()
            output.linear.x = command.linear
            output.angular.z = command.angular
            self.pod_publishers[pod].publish(output)
            if arrived:
                self._stop(pod)
                linear, angular = self.relative_speeds.get(pod, (float("inf"), float("inf")))
                if linear <= 0.02 and angular <= 0.08:
                    return True
            await self._sleep(period)
        self._stop(pod)
        return False

    def _docking_ready(
        self, pod: str, relative_target: list[float], latch_confirmed: bool
    ) -> tuple[bool, tuple[str, ...]]:
        estimate = self.estimates[pod]
        if not bool(self.get_parameter("require_connector_visibility").value):
            estimate = RelativePoseEstimate(
                estimate.pod_id, estimate.timestamp, estimate.pose,
                estimate.variance_x, estimate.variance_y, estimate.variance_yaw,
                estimate.uncertainty_class, True, estimate.sources,
                estimate.sensing_revision,
            )
        linear, angular = self.relative_speeds.get(pod, (float("inf"), float("inf")))
        return docking_acceptance(DockingEvidence(
            estimate, Pose2(*(float(value) for value in relative_target)),
            linear, angular, 1.0, latch_confirmed,
        ),
            translation_tolerance=float(
                self.get_parameter("position_tolerance").value),
            yaw_tolerance=float(self.get_parameter("yaw_tolerance").value),
        )

    async def _wait_for_docking_ready(
        self, pod: str, relative_target: list[float], latch_confirmed: bool,
        timeout: float,
    ) -> tuple[bool, tuple[str, ...]]:
        """Require consecutive accepted estimates after a physical transient."""
        deadline = time.monotonic() + timeout
        required = max(1, int(self.get_parameter("docking_stable_samples").value))
        stable = 0
        reasons: tuple[str, ...] = ("missing_observation",)
        previous_revision = -1
        while time.monotonic() < deadline:
            estimate = self.estimates.get(pod)
            if estimate is not None and estimate.sensing_revision != previous_revision:
                previous_revision = estimate.sensing_revision
                accepted, reasons = self._docking_ready(pod, relative_target, latch_confirmed)
                stable = stable + 1 if accepted else 0
                if stable >= required:
                    return True, ()
            await self._sleep(1.0 / float(self.get_parameter("control_rate").value))
        return False, reasons

    async def _begin(self, transition, goal):
        if not self.begin_client.wait_for_service(timeout_sec=2.0):
            return None
        request = BeginTransition.Request()
        request.transition_id = transition["id"]
        request.expected_source_morphology = goal.expected_source_morphology
        request.expected_topology_revision = goal.expected_topology_revision
        return await self.begin_client.call_async(request)

    async def _report_failure(self, transition_id: str, reason: str):
        if not self.fail_client.wait_for_service(timeout_sec=2.0):
            return None
        request = FailTransition.Request()
        request.transition_id = transition_id
        request.reason = reason
        return await self.fail_client.call_async(request)

    def _publish_connector_state(
        self, pod: str, morphology: str, latched: bool, detail: str
    ) -> None:
        message = ConnectorState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.pod_id = pod
        message.parent_port = f"{morphology}/{pod}" if latched else ""
        message.state = ConnectorState.LATCHED if latched else ConnectorState.DETACHED
        message.detail = detail
        self.state_pub.publish(message)

    async def _record_connector(
        self, pod: str, morphology: str, latched: bool, detail: str
    ) -> bool:
        if not self.observe_client.wait_for_service(timeout_sec=2.0):
            return False
        request = ObserveConnector.Request()
        request.pod_id = pod
        request.parent_port = f"{morphology}/{pod}" if latched else ""
        request.latched = latched
        request.detail = detail
        response = await self.observe_client.call_async(request)
        if response.success:
            self._publish_connector_state(pod, morphology, latched, detail)
        return bool(response.success)

    async def _commit(self, transition, expected_topology_revision: int):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            return None
        request = SetLocomotionMode.Request()
        request.expected_source_morphology = transition["from"]
        request.target_morphology = transition["to"]
        request.transition_id = transition["id"]
        request.expected_topology_revision = expected_topology_revision
        return await self.mode_client.call_async(request)

    async def _sleep(self, duration: float) -> None:
        future = Future()
        timer = None

        def wake() -> None:
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(duration, wake, callback_group=self.group)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    @staticmethod
    def _feedback(goal_handle, stage, pod, index, total):
        feedback = ExecuteReconfiguration.Feedback()
        feedback.stage = stage
        feedback.active_pod = pod
        feedback.progress = float(index) / max(1, total)
        goal_handle.publish_feedback(feedback)

    def _stop(self, pod: str) -> None:
        self.pod_publishers[pod].publish(Twist())

    def _stop_all(self) -> None:
        for pod in self.pod_publishers:
            self._stop(pod)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReconfigurationExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        if rclpy.ok():
            node._stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
