from __future__ import annotations

import asyncio
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from modular_robot_msgs.action import ExecuteReconfiguration
from modular_robot_msgs.msg import ConnectorCommand
from modular_robot_msgs.srv import SetLocomotionMode
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .control import Pose2, compose_pose, docking_command


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
        self.declare_parameter("relocation_timeout", 20.0)
        self.declare_parameter("control_rate", 30.0)
        self.declare_parameter("position_tolerance", 0.025)
        self.declare_parameter("yaw_tolerance", 0.08)
        self.declare_parameter("max_pod_linear", 0.22)
        self.declare_parameter("max_pod_angular", 1.0)
        with open(self.get_parameter("catalog").value, encoding="utf-8") as stream:
            self.catalog = yaml.safe_load(stream)

        self.group = ReentrantCallbackGroup()
        self.poses: dict[str, Pose2] = {}
        self.pose_times: dict[str, float] = {}
        self.joint_events: dict[str, tuple[str, int]] = {}
        self.event_counter = 0
        self.command_pub = self.create_publisher(ConnectorCommand, "connector_command", 10)
        self.transport_pub = self.create_publisher(String, "topology_joint_command", 10)
        self.create_subscription(
            String, "topology_joint_state", self._on_joint_state, 20,
            callback_group=self.group,
        )
        self.create_subscription(
            String, "module_pose", self._on_module_pose, 50,
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

    def _on_module_pose(self, message: String) -> None:
        parts = message.data.split("|")
        if len(parts) != 5 or parts[0] != "pose":
            return
        try:
            self.poses[parts[1]] = Pose2(float(parts[2]), float(parts[3]), float(parts[4]))
            self.pose_times[parts[1]] = time.monotonic()
        except ValueError:
            self.get_logger().warning(f"invalid module pose: {message.data}")

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
        if not await self._set_drive_enabled(False):
            result.message = "assembly drive adapter unavailable"
            goal_handle.abort()
            return result

        started = time.monotonic()
        try:
            if not await self._wait_for_pose("core"):
                raise RuntimeError("no core pose feedback")
            pods = transition.get("moved_pods", [])
            for index, pod in enumerate(pods):
                if goal_handle.is_cancel_requested:
                    raise asyncio.CancelledError
                self._feedback(goal_handle, "unlatch", pod, index, len(pods))
                since = self.event_counter
                await self._command_joint(pod, ConnectorCommand.DETACH, transition["from"])
                if not await self._wait_for_joint(pod, "detached", since):
                    raise RuntimeError(f"{pod} detach was not confirmed")

                self._feedback(goal_handle, "relocate", pod, index, len(pods))
                target = self.catalog["morphologies"][transition["to"]]["pods"][pod]
                if not await self._move_pod(pod, target, goal_handle):
                    raise RuntimeError(f"{pod} did not reach its docking pose")

                self._feedback(goal_handle, "latch", pod, index, len(pods))
                since = self.event_counter
                await self._command_joint(pod, ConnectorCommand.ATTACH, transition["to"])
                if not await self._wait_for_joint(pod, "attached", since):
                    raise RuntimeError(f"{pod} attach was not confirmed")

            self._feedback(goal_handle, "commit", "", len(pods), len(pods))
            response = await self._commit(transition)
            if response is None or not response.success:
                message = response.message if response else "morphology manager unavailable"
                raise RuntimeError(message)
        except asyncio.CancelledError:
            result.message = "transition cancelled; topology not committed"
            goal_handle.canceled()
            return result
        except RuntimeError as exc:
            result.message = f"transition failed; topology not committed: {exc}"
            goal_handle.abort()
            return result
        finally:
            self._stop_all()
            await self._set_drive_enabled(True)

        result.success = True
        result.message = "transition completed and physics topology verified"
        result.resulting_morphology = transition["to"]
        result.observed_time = time.monotonic() - started
        result.observed_energy = float(transition["energy"])
        goal_handle.succeed()
        return result

    async def _set_drive_enabled(self, enabled: bool) -> bool:
        if not self.drive_client.wait_for_service(timeout_sec=2.0):
            return False
        request = SetBool.Request()
        request.data = enabled
        response = await self.drive_client.call_async(request)
        return bool(response.success)

    async def _command_joint(self, pod: str, operation: int, morphology: str) -> None:
        command = ConnectorCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.operation = operation
        command.pod_id = pod
        command.parent_port = f"{morphology}/{pod}"
        self.command_pub.publish(command)
        transport = String()
        name = "detach" if operation == ConnectorCommand.DETACH else "attach"
        transport.data = f"{name}|{pod}"
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

    async def _move_pod(self, pod: str, relative_target: list[float], goal_handle) -> bool:
        if not await self._wait_for_pose(pod):
            return False
        deadline = time.monotonic() + float(self.get_parameter("relocation_timeout").value)
        period = 1.0 / float(self.get_parameter("control_rate").value)
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                raise asyncio.CancelledError
            core = self.poses.get("core")
            current = self.poses.get(pod)
            if core is None or current is None:
                await self._sleep(period)
                continue
            target = compose_pose(core, relative_target)
            command, arrived = docking_command(
                current, target,
                float(self.get_parameter("position_tolerance").value),
                float(self.get_parameter("yaw_tolerance").value),
                float(self.get_parameter("max_pod_linear").value),
                float(self.get_parameter("max_pod_angular").value),
            )
            output = Twist()
            output.linear.x = command.linear
            output.angular.z = command.angular
            self.pod_publishers[pod].publish(output)
            if arrived:
                self._stop(pod)
                return True
            await self._sleep(period)
        self._stop(pod)
        return False

    async def _commit(self, transition):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            return None
        request = SetLocomotionMode.Request()
        request.expected_source_morphology = transition["from"]
        request.target_morphology = transition["to"]
        request.transition_id = transition["id"]
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
        node._stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
