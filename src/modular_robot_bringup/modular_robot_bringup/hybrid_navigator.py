from __future__ import annotations

import hashlib
import json
from math import atan2, hypot
import time

import rclpy
from geometry_msgs.msg import PolygonStamped, PoseStamped, Twist
from modular_robot_msgs.action import ComputeHybridPlan, ExecuteReconfiguration, NavigateHybrid
from modular_robot_msgs.msg import HybridSegment, MorphologyState, RelativePoseEstimate
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from .qualification import (
    PlanarPose, goal_position_reached, motion_delta, qualification_pass,
)


class HybridNavigator(Node):
    def __init__(self) -> None:
        super().__init__("hybrid_navigator")
        self.group = ReentrantCallbackGroup()
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "core/base_link")
        self.declare_parameter("max_replans", 3)
        self.declare_parameter("costmap_footprint_timeout", 3.0)
        self.declare_parameter("costmap_footprint_padding", 0.01)
        self.declare_parameter("qualify_after_reconfiguration", True)
        self.declare_parameter("terminal_position_tolerance", 0.15)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.morphology: MorphologyState | None = None
        self.sensing_revision = 0
        self.costmap_footprints: dict[str, tuple[float, ...]] = {}
        self.latest_odometry: Odometry | None = None
        self.create_subscription(MorphologyState, "morphology_state", self._on_morphology, qos)
        self.create_subscription(
            RelativePoseEstimate, "relative_pose_estimate", self._on_sensing, 20)
        self.create_subscription(
            Odometry, "/odom", lambda message: setattr(self, "latest_odometry", message), 20)
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.drive_enable = self.create_client(
            SetBool, "set_assembly_drive_enabled", callback_group=self.group)
        self.create_subscription(
            PolygonStamped, "/local_costmap/published_footprint",
            lambda message: self._on_costmap_footprint("local", message), 10)
        self.create_subscription(
            PolygonStamped, "/global_costmap/published_footprint",
            lambda message: self._on_costmap_footprint("global", message), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.planner = ActionClient(self, ComputeHybridPlan, "compute_hybrid_plan", callback_group=self.group)
        self.follower = ActionClient(self, FollowPath, "follow_path", callback_group=self.group)
        self.reconfigure = ActionClient(self, ExecuteReconfiguration, "execute_reconfiguration", callback_group=self.group)
        self.server = ActionServer(self, NavigateHybrid, "navigate_hybrid", self._execute,
                                   callback_group=self.group)

    def _on_morphology(self, message):
        self.morphology = message

    def _on_sensing(self, message):
        # The estimator owns this global revision. Reconstructing it from local
        # callback order lets the navigator and planner disagree during startup
        # when their subscriptions receive pod samples in different orders.
        self.sensing_revision = max(
            self.sensing_revision, int(message.sensing_revision))

    def _on_costmap_footprint(self, name: str, message: PolygonStamped) -> None:
        self.costmap_footprints[name] = self._footprint_shape_signature(
            message.polygon.points)

    async def _execute(self, goal_handle):
        result = NavigateHybrid.Result()
        started = time.monotonic()
        reconfigurations = 0
        planning_latency = 0.0
        expanded_states = 0
        selected_plans = []
        execution_failures = 0
        for attempt in range(int(self.get_parameter("max_replans").value) + 1):
            if self.morphology is None:
                result.message = "no morphology state received"
                goal_handle.abort()
                return result
            if self.morphology.execution_state != MorphologyState.READY:
                result.message = self._unsafe_state_message()
                self._set_plan_metrics(
                    result, selected_plans, planning_latency, expanded_states)
                goal_handle.abort()
                return result
            start = self._current_pose()
            if start is None:
                result.message = "map-to-base transform unavailable"
                goal_handle.abort()
                return result
            planning_started = time.monotonic()
            plan_result = await self._plan(
                start, goal_handle.request.goal, goal_handle.request.planner_method)
            planning_latency += time.monotonic() - planning_started
            if (plan_result is not None and not plan_result.success
                    and "changed before planning" in plan_result.message):
                continue
            if plan_result is None or not plan_result.success:
                planning_message = plan_result.message if plan_result else "planner unavailable"
                result.message = (
                    f"execution failed; replanning failed: {planning_message}"
                    if execution_failures else planning_message)
                self._set_plan_metrics(
                    result, selected_plans, planning_latency, expanded_states)
                result.observed_time = time.monotonic() - started
                result.reconfiguration_count = reconfigurations
                goal_handle.abort()
                return result
            plan = plan_result.plan
            expanded_states += int(plan.expanded_states)
            selected_plans.append(plan)
            execution_failed = False
            replan_requested = False
            for index, segment in enumerate(plan.segments):
                if self.morphology.execution_state != MorphologyState.READY:
                    result.message = self._unsafe_state_message()
                    self._set_plan_metrics(
                        result, selected_plans, planning_latency, expanded_states)
                    goal_handle.abort()
                    return result
                if self.morphology.topology_revision != plan.topology_revision:
                    replan_requested = True
                    break
                if self.sensing_revision != plan.sensing_revision:
                    replan_requested = True
                    break
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
                    qualification_passed = True
                    if (success and bool(self.get_parameter(
                            "qualify_after_reconfiguration").value)):
                        qualification_passed, qualification = (
                            await self._qualify_assembled_motion())
                        result.motion_qualification_json = [
                            *result.motion_qualification_json,
                            json.dumps(qualification, sort_keys=True),
                        ]
                    if success and not qualification_passed:
                        await self._inhibit_assembly_drive()
                        result.message = (
                            "reconfiguration committed; post-transition assembled "
                            "motion qualification failed; drive inhibited")
                        result.observed_time = time.monotonic() - started
                        result.reconfiguration_count = reconfigurations
                        self._set_plan_metrics(
                            result, selected_plans, planning_latency, expanded_states)
                        goal_handle.abort()
                        return result
                    if success and not await self._wait_for_costmap_footprints():
                        result.message = (
                            "reconfiguration committed; costmaps did not acknowledge "
                            "the new morphology footprint"
                        )
                        result.observed_time = time.monotonic() - started
                        result.reconfiguration_count = reconfigurations
                        self._set_plan_metrics(
                            result, selected_plans, planning_latency, expanded_states)
                        goal_handle.abort()
                        return result
                    replan_requested = bool(success)
                else:
                    success = await self._follow(segment)
                if not success:
                    if (segment.kind == HybridSegment.RECONFIGURE and self.morphology
                            and self.morphology.execution_state
                            == MorphologyState.RECOVERY_REQUIRED):
                        result.message = (
                            "reconfiguration failed; observed topology requires recovery"
                        )
                        result.observed_time = time.monotonic() - started
                        result.reconfiguration_count = reconfigurations
                        self._set_plan_metrics(
                            result, selected_plans, planning_latency, expanded_states)
                        goal_handle.abort()
                        return result
                    execution_failed = True
                    break
                if replan_requested:
                    break
            if not execution_failed:
                if replan_requested:
                    continue
                if not self._terminal_position_reached(goal_handle.request.goal):
                    self.get_logger().error(
                        "path follower reported success outside terminal position tolerance")
                    execution_failed = True
                else:
                    result.success = True
                    result.message = "hybrid navigation completed"
                    result.observed_time = time.monotonic() - started
                    result.reconfiguration_count = reconfigurations
                    self._set_plan_metrics(
                        result, selected_plans, planning_latency, expanded_states)
                    goal_handle.succeed()
                    return result
            self.get_logger().warning(f"execution failed; replanning attempt {attempt + 1}")
            execution_failures += 1
        result.message = "execution failed after replanning limit"
        result.observed_time = time.monotonic() - started
        result.reconfiguration_count = reconfigurations
        self._set_plan_metrics(result, selected_plans, planning_latency, expanded_states)
        goal_handle.abort()
        return result

    def _unsafe_state_message(self) -> str:
        state = self.morphology.execution_state if self.morphology else -1
        names = {
            MorphologyState.TRANSITIONING: "transitioning",
            MorphologyState.RECOVERY_REQUIRED: "recovery required",
            MorphologyState.STOPPED: "stopped",
        }
        return f"navigation blocked; observed topology state is {names.get(state, state)}"

    @staticmethod
    def _footprint_shape_signature(points) -> tuple[float, ...]:
        """Edge lengths identify a rigid polygon independent of costmap frame."""
        coordinates = [
            (point.x, point.y) if hasattr(point, "x") else point for point in points
        ]
        return tuple(round(hypot(
            coordinates[(index + 1) % len(coordinates)][0] - point[0],
            coordinates[(index + 1) % len(coordinates)][1] - point[1],
        ), 3) for index, point in enumerate(coordinates))

    def _expected_costmap_footprint(self) -> tuple[float, ...]:
        points = self.morphology.footprint.points
        padding = float(self.get_parameter("costmap_footprint_padding").value)
        center_x = sum(point.x for point in points) / len(points)
        center_y = sum(point.y for point in points) / len(points)
        padded = []
        for point in points:
            # Confirmatory footprints are axis-aligned rectangles. Nav2 expands
            # each side by footprint_padding before publishing it in a world frame.
            x = point.x + (padding if point.x > center_x else -padding)
            y = point.y + (padding if point.y > center_y else -padding)
            padded.append((x, y))
        return self._footprint_shape_signature(padded)

    async def _wait_for_costmap_footprints(self) -> bool:
        expected = self._expected_costmap_footprint()
        deadline = time.monotonic() + float(
            self.get_parameter("costmap_footprint_timeout").value)
        while time.monotonic() < deadline:
            if all(self.costmap_footprints.get(name) == expected
                   for name in ("local", "global")):
                return True
            await self._sleep(0.05)
        observed = {name: self.costmap_footprints.get(name)
                    for name in ("local", "global")}
        self.get_logger().error(
            f"costmap footprint acknowledgement timeout; expected={expected}; "
            f"observed={observed}")
        return False

    async def _sleep(self, duration: float) -> None:
        future = Future()

        def wake() -> None:
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(duration, wake, callback_group=self.group)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    @staticmethod
    def _set_plan_metrics(result, plans, planning_latency, expanded_states):
        result.planning_latency = planning_latency
        result.expanded_states = expanded_states
        if not plans:
            return
        final = plans[-1]
        result.map_revision = final.map_revision
        result.topology_revision = final.topology_revision
        result.sensing_revision = final.sensing_revision
        transitions = []
        signature_parts = []
        for plan in plans:
            for segment in plan.segments:
                poses = segment.path.poses if segment.kind == HybridSegment.TRAVERSE else [segment.execution_pose]
                signature_parts.extend(
                    f"{segment.kind}:{segment.morphology_id}:{pose.pose.position.x:.3f}:"
                    f"{pose.pose.position.y:.3f}:{segment.transition_id}"
                    for pose in poses)
                if segment.kind == HybridSegment.RECONFIGURE:
                    transitions.append(segment)
        result.planned_route_signature = hashlib.sha256(
            "|".join(signature_parts).encode()).hexdigest()
        result.planned_transition_ids = [segment.transition_id for segment in transitions]
        result.planned_transition_poses = [segment.execution_pose for segment in transitions]
        route_poses = []
        for plan in plans:
            for segment in plan.segments:
                if segment.kind != HybridSegment.TRAVERSE:
                    continue
                for pose in segment.path.poses:
                    if (not route_poses or
                            pose.pose.position.x != route_poses[-1].pose.position.x or
                            pose.pose.position.y != route_poses[-1].pose.position.y):
                        route_poses.append(pose)
        result.planned_route_poses = route_poses

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

    def _terminal_position_reached(self, goal: PoseStamped) -> bool:
        current = self._current_pose()
        if current is None:
            return False
        return goal_position_reached(
            PlanarPose(current.pose.position.x, current.pose.position.y, 0.0),
            PlanarPose(goal.pose.position.x, goal.pose.position.y, 0.0),
            float(self.get_parameter("terminal_position_tolerance").value),
        )

    async def _plan(self, start, goal, planner_method):
        if not self.planner.wait_for_server(timeout_sec=3.0):
            return None
        request = ComputeHybridPlan.Goal()
        request.start = start
        request.goal = goal
        request.start_morphology = self.morphology.morphology_id
        request.initial_epsilon = 2.5
        request.planner_method = planner_method
        request.expected_topology_revision = self.morphology.topology_revision
        request.expected_sensing_revision = self.sensing_revision
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

    @staticmethod
    def _pose_from_odometry(message: Odometry) -> PlanarPose:
        pose = message.pose.pose
        quaternion = pose.orientation
        yaw = 2.0 * atan2(quaternion.z, quaternion.w)
        return PlanarPose(float(pose.position.x), float(pose.position.y), yaw)

    async def _command_for(self, linear: float, angular: float,
                           duration: float) -> tuple[PlanarPose, PlanarPose] | None:
        if self.latest_odometry is None:
            return None
        start = self._pose_from_odometry(self.latest_odometry)
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.command_publisher.publish(command)
            await self._sleep(0.05)
        self.command_publisher.publish(Twist())
        await self._sleep(0.25)
        if self.latest_odometry is None:
            return None
        return start, self._pose_from_odometry(self.latest_odometry)

    async def _qualify_assembled_motion(self) -> tuple[bool, dict]:
        sequence = (
            ("positive_yaw", 0.0, 0.25, 1.5),
            ("negative_yaw", 0.0, -0.25, 1.5),
            ("forward", 0.12, 0.0, 1.0),
            ("reverse", -0.12, 0.0, 1.0),
        )
        stages = {}
        for name, linear, angular, duration in sequence:
            endpoints = await self._command_for(linear, angular, duration)
            if endpoints is None:
                return False, {"passed": False, "reason": "odometry_unavailable"}
            stages[name] = motion_delta(*endpoints)
        passed = qualification_pass(stages)
        log = self.get_logger().info if passed else self.get_logger().error
        log(f"post-transition motion qualification passed={passed}; stages={stages}")
        return passed, {"passed": passed, "stages": stages}

    async def _inhibit_assembly_drive(self) -> None:
        self.command_publisher.publish(Twist())
        if not self.drive_enable.wait_for_service(timeout_sec=1.0):
            return
        request = SetBool.Request()
        request.data = False
        await self.drive_enable.call_async(request)


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
