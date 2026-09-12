from __future__ import annotations

from math import atan2, cos, sin

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from modular_robot_msgs.action import ComputeHybridPlan
from modular_robot_msgs.msg import HybridPlan as HybridPlanMsg
from modular_robot_msgs.msg import HybridSegment as HybridSegmentMsg
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from rclpy.action import ActionServer
from rclpy.node import Node

from .catalog import load_catalog
from .cost_model import LinearCalibratedCostModel, OnnxCostModel
from .grid import OccupancyGrid
from .planner import HybridState, MorphologyAStar, NoPathError


class PlannerServer(Node):
    def __init__(self) -> None:
        super().__init__("morphology_planner")
        default_catalog = (
            get_package_share_directory("modular_robot_description")
            + "/config/morphologies.yaml"
        )
        self.declare_parameter("catalog", default_catalog)
        self.declare_parameter("heading_bins", 8)
        self.declare_parameter("planning_resolution", 0.1)
        self.declare_parameter("cost_model", "")
        self._catalog = load_catalog(self.get_parameter("catalog").value)
        self._grid: OccupancyGrid | None = None
        self._map_sub = self.create_subscription(
            OccupancyGridMsg, "/map", self._on_map, 1
        )
        self._server = ActionServer(
            self, ComputeHybridPlan, "compute_hybrid_plan", self._execute
        )

    def _on_map(self, message: OccupancyGridMsg) -> None:
        native = OccupancyGrid(
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
            data=list(message.data),
            revision=(self._grid.revision + 1) if self._grid else 1,
        )
        self._grid = native.coarsen(float(self.get_parameter("planning_resolution").value))

    async def _execute(self, goal_handle):
        result = ComputeHybridPlan.Result()
        if self._grid is None:
            result.success = False
            result.message = "no occupancy map received"
            goal_handle.abort()
            return result

        request = goal_handle.request
        planner = MorphologyAStar(
            self._catalog, self._grid,
            heading_bins=self.get_parameter("heading_bins").value,
            cost_model=self._cost_model(),
        )
        sx, sy = self._grid.world_to_cell(
            request.start.pose.position.x, request.start.pose.position.y
        )
        gx, gy = self._grid.world_to_cell(
            request.goal.pose.position.x, request.goal.pose.position.y
        )
        heading = _heading_bin(request.start, planner.heading_bins)
        try:
            epsilon = max(1.0, float(request.initial_epsilon or 2.5))
            plans = planner.plan_anytime(
                HybridState(sx, sy, heading, request.start_morphology),
                (gx, gy),
                (epsilon,),
            )
        except (NoPathError, ValueError) as exc:
            result.success = False
            result.message = str(exc)
            goal_handle.abort()
            return result

        plan = plans[-1]
        result.plan = _to_message(
            plan, self._grid, self._catalog,
            request.start.header.frame_id or "map", planner.heading_bins,
        )
        result.success = True
        result.message = "hybrid plan found"
        goal_handle.succeed()
        return result

    def _cost_model(self):
        path = str(self.get_parameter("cost_model").value)
        if not path:
            return None
        return OnnxCostModel(path) if path.endswith(".onnx") else LinearCalibratedCostModel(path)


def _heading_bin(pose: PoseStamped, bins: int) -> int:
    q = pose.pose.orientation
    yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return round((yaw % (2.0 * 3.141592653589793)) * bins / (2.0 * 3.141592653589793)) % bins


def _pose_for(state, grid: OccupancyGrid, frame_id: str, heading_bins: int) -> PoseStamped:
    output = PoseStamped()
    output.header.frame_id = frame_id
    output.pose.position.x, output.pose.position.y = grid.cell_center(state.x, state.y)
    yaw = state.heading * 2.0 * 3.141592653589793 / heading_bins
    output.pose.orientation.z = sin(yaw / 2.0)
    output.pose.orientation.w = cos(yaw / 2.0)
    return output


def _to_message(plan, grid: OccupancyGrid, catalog, frame_id: str, heading_bins: int) -> HybridPlanMsg:
    message = HybridPlanMsg()
    message.header.frame_id = frame_id
    message.plan_id = f"map-{plan.map_revision}-expanded-{plan.expanded}"
    message.total_cost = plan.total_cost
    message.expanded_states = plan.expanded
    message.epsilon = plan.epsilon
    message.map_revision = plan.map_revision
    for segment in plan.segments:
        if (segment.kind == "traverse" and message.segments
                and message.segments[-1].kind == HybridSegmentMsg.TRAVERSE
                and message.segments[-1].morphology_id == segment.target.morphology):
            previous = message.segments[-1]
            previous.path.poses.append(_pose_for(segment.target, grid, frame_id, heading_bins))
            previous.objective_cost += segment.cost
            continue
        output = HybridSegmentMsg()
        output.kind = output.RECONFIGURE if segment.kind == "reconfigure" else output.TRAVERSE
        output.morphology_id = segment.target.morphology
        output.controller_id = catalog.morphologies[segment.target.morphology].controller_id
        output.transition_id = segment.transition_id
        output.objective_cost = segment.cost
        output.execution_pose = _pose_for(segment.source, grid, frame_id, heading_bins)
        if output.kind == output.TRAVERSE:
            output.path.header.frame_id = frame_id
            output.path.poses.append(_pose_for(segment.source, grid, frame_id, heading_bins))
            output.path.poses.append(_pose_for(segment.target, grid, frame_id, heading_bins))
        else:
            transition = next(t for t in catalog.transitions if t.id == segment.transition_id)
            output.predicted_time = transition.time
            output.predicted_energy = transition.energy
            output.predicted_failure_probability = transition.failure_probability
        message.segments.append(output)
    return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
