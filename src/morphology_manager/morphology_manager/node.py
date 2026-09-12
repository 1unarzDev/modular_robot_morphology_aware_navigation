from __future__ import annotations

import hashlib
import json

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point32, Polygon
from modular_robot_msgs.msg import ModuleConnection, MorphologyState
from modular_robot_msgs.srv import (
    BeginTransition, FailTransition, ObserveConnector, ReconcileTopology,
    SetLocomotionMode,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .state import ExecutionState, TopologyStateMachine


class MorphologyManager(Node):
    def __init__(self) -> None:
        super().__init__("morphology_manager")
        default_catalog = get_package_share_directory("modular_robot_description") + "/config/morphologies.yaml"
        self.declare_parameter("catalog", default_catalog)
        self.declare_parameter("initial_morphology", "compact_diff")
        with open(self.get_parameter("catalog").value, encoding="utf-8") as stream:
            self.catalog = yaml.safe_load(stream)
        initial = str(self.get_parameter("initial_morphology").value)
        if initial not in self.catalog["morphologies"]:
            raise ValueError(f"unknown initial morphology: {initial}")
        self.machine = TopologyStateMachine(self.catalog, initial)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(MorphologyState, "morphology_state", qos)
        # Nav2 Jazzy subscribes to Polygon on its footprint input topic.
        self.local_footprint = self.create_publisher(Polygon, "/local_costmap/footprint", qos)
        self.global_footprint = self.create_publisher(Polygon, "/global_costmap/footprint", qos)
        self.service = self.create_service(SetLocomotionMode, "set_locomotion_mode", self._set_mode)
        self.begin_service = self.create_service(BeginTransition, "begin_transition", self._begin)
        self.fail_service = self.create_service(FailTransition, "fail_transition", self._fail)
        self.reconcile_service = self.create_service(
            ReconcileTopology, "reconcile_topology", self._reconcile
        )
        self.observe_service = self.create_service(
            ObserveConnector, "observe_connector", self._observe
        )
        self.create_timer(1.0, self.publish_state)
        self.publish_state()

    def _set_mode(self, request, response):
        valid_transition = next((t for t in self.catalog["transitions"]
            if t["id"] == request.transition_id
            and t["to"] == request.target_morphology), None)
        if valid_transition is None:
            response.success = False
            response.message = "transition is not in the validated catalog"
        else:
            try:
                if request.expected_topology_revision != self.machine.topology_revision:
                    raise ValueError(f"stale topology revision; observed {self.machine.topology_revision}")
                self.machine.commit(valid_transition)
                response.success = True
                response.message = "observed topology committed"
            except ValueError as exc:
                self.machine.fail()
                response.success = False
                response.message = str(exc)
        response.state = self._state()
        self.publish_state()
        return response

    def _begin(self, request, response):
        transition = next((t for t in self.catalog["transitions"] if t["id"] == request.transition_id), None)
        try:
            if transition is None:
                raise ValueError("unknown transition")
            self.machine.begin(transition, request.expected_source_morphology,
                               request.expected_topology_revision)
            response.success, response.message = True, "transition interlock acquired"
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        response.state = self._state(); self.publish_state()
        return response

    def _fail(self, request, response):
        if self.machine.active_transition not in (None, request.transition_id):
            response.success, response.message = False, "failure report does not match active transition"
        else:
            self.machine.fail()
            response.success, response.message = True, f"recovery required: {request.reason}"
        response.state = self._state(); self.publish_state()
        return response

    def _reconcile(self, request, response):
        try:
            morphology = self.machine.reconcile()
            response.success, response.message = True, f"reconciled as {morphology}"
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        response.state = self._state(); self.publish_state()
        return response

    def _observe(self, request, response):
        if request.pod_id not in self.catalog["inventory"]:
            response.success, response.message = False, "unknown module"
        elif self.machine.execution_state != ExecutionState.TRANSITIONING:
            response.success, response.message = False, "no active transition"
        else:
            self.machine.observe(request.pod_id, request.parent_port, request.latched)
            response.success, response.message = True, "connector observation recorded"
            self.publish_state()
        response.topology_revision = self.machine.topology_revision
        return response

    def _state(self) -> MorphologyState:
        value = self.catalog["morphologies"][self.machine.morphology_id]
        message = MorphologyState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.catalog["frame_id"]
        message.morphology_id = self.machine.morphology_id
        message.locomotion_mode = value["locomotion_mode"]
        message.controller_id = value["controller_id"]
        message.height = float(value["height"])
        for x, y in value["footprint"]:
            message.footprint.points.append(Point32(x=float(x), y=float(y), z=0.0))
        modules = list(self.catalog["inventory"].keys())
        message.topology.module_ids = modules
        message.topology.module_types = [self.catalog["inventory"][m]["type"] for m in modules]
        connections = []
        for pod_id, observed in sorted(self.machine.connections.items()):
            morphology, _ = observed.parent_port.split("/", 1)
            pose = self.catalog["morphologies"][morphology]["pods"][pod_id]
            connection = ModuleConnection()
            connection.parent_module = "core"
            connection.parent_port = observed.parent_port
            connection.child_module = pod_id
            connection.child_port = "dock"
            connection.transform.translation.x = float(pose[0])
            connection.transform.translation.y = float(pose[1])
            connection.transform.rotation.w = 1.0
            connection.latched = True
            connections.append(connection)
        message.topology.connections = connections
        topology = [(c.parent_port, c.child_module) for c in connections]
        message.topology.graph_hash = hashlib.sha256(
            json.dumps(topology, sort_keys=True).encode("utf-8")
        ).hexdigest()[:16]
        message.transition_active = self.machine.execution_state == ExecutionState.TRANSITIONING
        message.execution_state = int(self.machine.execution_state)
        message.topology_revision = self.machine.topology_revision
        message.sensing_revision = self.machine.sensing_revision
        return message

    def publish_state(self) -> None:
        state = self._state()
        self.publisher.publish(state)
        self.local_footprint.publish(state.footprint)
        self.global_footprint.publish(state.footprint)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MorphologyManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
