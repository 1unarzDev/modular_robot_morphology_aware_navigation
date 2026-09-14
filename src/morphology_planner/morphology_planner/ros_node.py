from __future__ import annotations

import hashlib
import json
from math import atan2, cos, sin

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from modular_robot_msgs.action import ComputeHybridPlan
from modular_robot_msgs.msg import HybridPlan as HybridPlanMsg
from modular_robot_msgs.msg import HybridSegment as HybridSegmentMsg
from modular_robot_msgs.msg import MorphologyState, RelativePoseEstimate
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .catalog import load_catalog
from .cost_model import LinearCalibratedCostModel, OnnxCostModel
from .grid import OccupancyGrid
from .methods import make_method_planner
from .planner import HybridState, NoPathError
from .transition_policy import PodSensingState


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
        self.declare_parameter("experiment_supported_only", True)
        self.declare_parameter("planner_method", "sensing_feasibility_coupled")
        catalog = load_catalog(self.get_parameter("catalog").value)
        self._catalog = (
            catalog.supported_experiment_subset()
            if self.get_parameter("experiment_supported_only").value else catalog
        )
        self._grid: OccupancyGrid | None = None
        self._map_signature = ""
        self._map_revision = 0
        self._topology_revision = 0
        self._sensing_revision = 0
        self._sensing: dict[str, PodSensingState] = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._map_sub = self.create_subscription(
            OccupancyGridMsg, "/map", self._on_map, qos
        )
        self.create_subscription(
            MorphologyState, "morphology_state", self._on_morphology, qos)
        self.create_subscription(
            RelativePoseEstimate, "relative_pose_estimate", self._on_sensing, 20)
        self._server = ActionServer(
            self, ComputeHybridPlan, "compute_hybrid_plan", self._execute
        )

    def _on_map(self, message: OccupancyGridMsg) -> None:
        digest = hashlib.sha256()
        digest.update(
            f"{message.info.width}|{message.info.height}|{message.info.resolution}|"
            f"{message.info.origin.position.x}|{message.info.origin.position.y}|".encode())
        digest.update(bytes((value + 1) & 0xff for value in message.data))
        signature = digest.hexdigest()
        if signature != self._map_signature:
            self._map_signature = signature
            self._map_revision += 1
        native = OccupancyGrid(
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
            data=list(message.data),
            revision=self._map_revision,
        )
        self._grid = native.coarsen(float(self.get_parameter("planning_resolution").value))

    def _on_morphology(self, message: MorphologyState) -> None:
        self._topology_revision = int(message.topology_revision)

    def _on_sensing(self, message: RelativePoseEstimate) -> None:
        # RelativePoseEstimate carries the estimator-owned global revision.
        # Use it directly so callback interleavings cannot create false stale
        # plan failures between this server and the navigator.
        self._sensing_revision = max(
            self._sensing_revision, int(message.sensing_revision))
        covariance = message.pose.covariance
        self._sensing[message.pod_id] = PodSensingState(
            bool(message.connector_visible),
            float(covariance[0] + covariance[7] + covariance[35]),
        )

    async def _execute(self, goal_handle):
        result = ComputeHybridPlan.Result()
        if self._grid is None:
            result.success = False
            result.message = "no occupancy map received"
            goal_handle.abort()
            return result

        request = goal_handle.request
        if (request.expected_topology_revision and
                request.expected_topology_revision != self._topology_revision):
            result.success = False
            result.message = "topology revision changed before planning"
            goal_handle.abort()
            return result
        if (request.expected_sensing_revision and
                request.expected_sensing_revision != self._sensing_revision):
            result.success = False
            result.message = "sensing signature changed before planning"
            goal_handle.abort()
            return result
        planner = None
        try:
            start_world = (
                request.start.pose.position.x, request.start.pose.position.y)
            goal_world = (
                request.goal.pose.position.x, request.goal.pose.position.y)
            planning_grid = self._grid.padded_to_include(
                (start_world, goal_world),
                margin=max(value.radius for value in self._catalog.morphologies.values()),
            )
            method = str(request.planner_method or self.get_parameter("planner_method").value)
            planner = make_method_planner(
                method, self._catalog, planning_grid,
                heading_bins=self.get_parameter("heading_bins").value,
                sensing=self._sensing,
                cost_model=self._cost_model(),
                topology_revision=self._topology_revision,
                sensing_revision=self._sensing_revision,
            )
            sx, sy = planning_grid.world_to_cell(
                request.start.pose.position.x, request.start.pose.position.y
            )
            gx, gy = planning_grid.world_to_cell(
                request.goal.pose.position.x, request.goal.pose.position.y
            )
            heading = _heading_bin(request.start, planner.planner.heading_bins
                                   if hasattr(planner.planner, "heading_bins")
                                   else planner.planner.hybrid.heading_bins)
            epsilon = max(1.0, float(request.initial_epsilon or 2.5))
            plan = planner.plan(
                HybridState(sx, sy, heading, request.start_morphology),
                (gx, gy), epsilon)
        except (NoPathError, ValueError) as exc:
            result.success = False
            result.message = str(exc)
            # Rejected edges explain a failed search, so keep them too.
            if planner is not None:
                result.transition_decision_json = [
                    json.dumps(row, sort_keys=True)
                    for row in planner.aggregated_transition_decisions()]
            goal_handle.abort()
            return result

        result.plan = _to_message(
            plan, planning_grid, self._catalog,
            request.start.header.frame_id or "map",
            planner.planner.heading_bins if hasattr(planner.planner, "heading_bins")
            else planner.planner.hybrid.heading_bins,
        )
        result.transition_decision_json = [
            json.dumps(row, sort_keys=True)
            for row in planner.aggregated_transition_decisions()]
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
    message.method_id = plan.method_id
    message.topology_revision = plan.topology_revision
    message.sensing_revision = plan.sensing_revision
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
