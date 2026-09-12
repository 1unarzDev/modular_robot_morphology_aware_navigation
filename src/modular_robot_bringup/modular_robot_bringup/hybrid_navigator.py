from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from modular_robot_msgs.action import ComputeHybridPlan, ExecuteReconfiguration, NavigateHybrid
from modular_robot_msgs.msg import HybridSegment, MorphologyState
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


class HybridNavigator(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_navigator")
        self.group = ReentrantCallbackGroup()
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "core/base_link")
        self.declare_parameter("max_replans", 3)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.morphology: MorphologyState | None = None
        self.create_subscription(MorphologyState, "morphology_state", self._on_morphology, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.planner = ActionClient(self, ComputeHybridPlan, "compute_hybrid_plan", callback_group=self.group)
        self.follower = ActionClient(self, FollowPath, "follow_path", callback_group=self.group)
        self.reconfigure = ActionClient(self, ExecuteReconfiguration, "execute_reconfiguration", callback_group=self.group)
        self.server = ActionServer(self, NavigateHybrid, "navigate_hybrid", self._execute,
                                   callback_group=self.group)

    def _on_morphology(self, message):
        self.morphology = message

    async def _execute(self, goal_handle):
        result = NavigateHybrid.Result()
        started = time.monotonic()
        reconfigurations = 0
        for attempt in range(int(self.get_parameter("max_replans").value) + 1):
            if self.morphology is None:
                result.message = "no morphology state received"
                goal_handle.abort()
                return result
            start = self._current_pose()
            if start is None:
                result.message = "map-to-base transform unavailable"
                goal_handle.abort()
                return result
            plan_result = await self._plan(start, goal_handle.request.goal)
            if plan_result is None or not plan_result.success:
                result.message = plan_result.message if plan_result else "planner unavailable"
                goal_handle.abort()
                return result
            plan = plan_result.plan
            execution_failed = False
            for index, segment in enumerate(plan.segments):
                feedback = NavigateHybrid.Feedback()
                feedback.stage = "reconfigure" if segment.kind == HybridSegment.RECONFIGURE else "traverse"
                feedback.segment_index = index
                feedback.segment_count = len(plan.segments)
                feedback.morphology_id = segment.morphology_id
                goal_handle.publish_feedback(feedback)
                if goal_handle.is_cancel_requested:
                    result.message = "navigation cancelled"
                    goal_handle.canceled()
                    return result
                if segment.kind == HybridSegment.RECONFIGURE:
                    success = await self._execute_transition(segment)
                    reconfigurations += 1
                else:
                    success = await self._follow(segment)
                if not success:
                    execution_failed = True
                    break
            if not execution_failed:
                result.success = True
                result.message = "hybrid navigation completed"
                result.observed_time = time.monotonic() - started
                result.reconfiguration_count = reconfigurations
                goal_handle.succeed()
                return result
            self.get_logger().warning(f"execution failed; replanning attempt {attempt + 1}")
        result.message = "execution failed after replanning limit"
        result.observed_time = time.monotonic() - started
        result.reconfiguration_count = reconfigurations
        goal_handle.abort()
        return result

    def _current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter("map_frame").value,
                self.get_parameter("base_frame").value,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warning(str(exc))
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    async def _plan(self, start, goal):
        if not self.planner.wait_for_server(timeout_sec=3.0):
            return None
        request = ComputeHybridPlan.Goal()
        request.start = start
        request.goal = goal
        request.start_morphology = self.morphology.morphology_id
        request.initial_epsilon = 2.5
        handle = await self.planner.send_goal_async(request)
        if not handle.accepted:
            return None
        return (await handle.get_result_async()).result

    async def _follow(self, segment):
        if not self.follower.wait_for_server(timeout_sec=3.0):
            return False
        request = FollowPath.Goal()
        request.path = segment.path
        request.controller_id = segment.controller_id
        handle = await self.follower.send_goal_async(request)
        if not handle.accepted:
            return False
        response = (await handle.get_result_async()).result
        return int(response.error_code) == 0

    async def _execute_transition(self, segment):
        if not self.reconfigure.wait_for_server(timeout_sec=3.0):
            return False
        request = ExecuteReconfiguration.Goal()
        request.transition_id = segment.transition_id
        request.execution_pose = segment.execution_pose
        request.expected_source_morphology = self.morphology.morphology_id
        request.expected_topology_revision = self.morphology.topology_revision
        handle = await self.reconfigure.send_goal_async(request)
        if not handle.accepted:
            return False
        return bool((await handle.get_result_async()).result.success)


def main(args=None):
    rclpy.init(args=args)
    node = HybridNavigator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
