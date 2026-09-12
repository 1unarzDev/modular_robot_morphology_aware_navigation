from __future__ import annotations

import argparse

import rclpy
from modular_robot_msgs.action import ExecuteReconfiguration
from modular_robot_msgs.msg import MorphologyState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ReconfigurationClient(Node):
    def __init__(self) -> None:
        super().__init__("reconfiguration_test_client")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.state: MorphologyState | None = None
        self.create_subscription(MorphologyState, "morphology_state", self._state, qos)
        self.client = ActionClient(self, ExecuteReconfiguration, "execute_reconfiguration")

    def _state(self, message: MorphologyState) -> None:
        self.state = message


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("transition_id")
    parser.add_argument("--timeout", type=float, default=90.0)
    parsed = parser.parse_args(args)
    rclpy.init()
    node = ReconfigurationClient()
    try:
        deadline = node.get_clock().now().nanoseconds / 1e9 + 5.0
        while node.state is None and node.get_clock().now().nanoseconds / 1e9 < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.state is None or not node.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("reconfiguration action or morphology state unavailable")
        request = ExecuteReconfiguration.Goal()
        request.transition_id = parsed.transition_id
        request.expected_source_morphology = node.state.morphology_id
        request.expected_topology_revision = node.state.topology_revision
        request.execution_pose.header.frame_id = "map"
        request.execution_pose.pose.orientation.w = 1.0
        goal_future = node.client.send_goal_async(request)
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=5.0)
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("reconfiguration goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=parsed.timeout)
        if not result_future.done():
            handle.cancel_goal_async()
            raise TimeoutError("reconfiguration action timed out")
        result = result_future.result().result
        print(f"success={result.success} morphology={result.resulting_morphology} "
              f"time={result.observed_time:.3f} message={result.message}")
        if not result.success:
            raise RuntimeError(result.message)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
