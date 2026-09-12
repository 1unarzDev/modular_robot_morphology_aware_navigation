from __future__ import annotations

import hashlib
import json

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point32, Polygon
from modular_robot_msgs.msg import ModuleConnection, MorphologyState
from modular_robot_msgs.srv import SetLocomotionMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MorphologyManager(Node):
    def __init__(self) -> None:
        super().__init__("morphology_manager")
        default_catalog = get_package_share_directory("modular_robot_description") + "/config/morphologies.yaml"
        self.declare_parameter("catalog", default_catalog)
        self.declare_parameter("initial_morphology", "compact_diff")
        with open(self.get_parameter("catalog").value, encoding="utf-8") as stream:
            self.catalog = yaml.safe_load(stream)
        self.current = str(self.get_parameter("initial_morphology").value)
        if self.current not in self.catalog["morphologies"]:
            raise ValueError(f"unknown initial morphology: {self.current}")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(MorphologyState, "morphology_state", qos)
        # Nav2 Jazzy subscribes to Polygon on its footprint input topic.
        self.local_footprint = self.create_publisher(Polygon, "/local_costmap/footprint", qos)
        self.global_footprint = self.create_publisher(Polygon, "/global_costmap/footprint", qos)
        self.service = self.create_service(SetLocomotionMode, "set_locomotion_mode", self._set_mode)
        self.create_timer(1.0, self.publish_state)
        self.publish_state()

    def _set_mode(self, request, response):
        valid_transition = next((t for t in self.catalog["transitions"]
            if t["id"] == request.transition_id and t["from"] == self.current
            and t["to"] == request.target_morphology), None)
        if request.expected_source_morphology != self.current:
            response.success = False
            response.message = f"stale source: expected {self.current}"
        elif valid_transition is None:
            response.success = False
            response.message = "transition is not in the certified catalog"
        else:
            self.current = request.target_morphology
            response.success = True
            response.message = "morphology committed"
        response.state = self._state()
        if response.success:
            self.publisher.publish(response.state)
        return response

    def _state(self) -> MorphologyState:
        value = self.catalog["morphologies"][self.current]
        message = MorphologyState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.catalog["frame_id"]
        message.morphology_id = self.current
        message.locomotion_mode = value["locomotion_mode"]
        message.controller_id = value["controller_id"]
        message.height = float(value["height"])
        for x, y in value["footprint"]:
            message.footprint.points.append(Point32(x=float(x), y=float(y), z=0.0))
        modules = list(self.catalog["inventory"].keys())
        message.topology.module_ids = modules
        message.topology.module_types = [self.catalog["inventory"][m]["type"] for m in modules]
        connections = []
        for pod_id, pose in value["pods"].items():
            connection = ModuleConnection()
            connection.parent_module = "core"
            connection.parent_port = f"{self.current}/{pod_id}"
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
