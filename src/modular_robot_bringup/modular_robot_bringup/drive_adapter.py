from __future__ import annotations

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from modular_robot_msgs.msg import MorphologyState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from .kinematics import BodyTwist, distribute_twist


class AssemblyDriveAdapter(Node):
    """Maps the active morphology's body command to attached drive pods."""

    def __init__(self) -> None:
        super().__init__("assembly_drive_adapter")
        default_catalog = (
            get_package_share_directory("modular_robot_description")
            + "/config/morphologies.yaml"
        )
        self.declare_parameter("catalog", default_catalog)
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("command_timeout", 0.3)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("initial_morphology", "compact_diff")
        with open(self.get_parameter("catalog").value, encoding="utf-8") as stream:
            self.catalog = yaml.safe_load(stream)

        self.enabled = True
        self.morphology = str(self.get_parameter("initial_morphology").value)
        if self.morphology not in self.catalog["morphologies"]:
            raise ValueError(f"unknown initial morphology: {self.morphology}")
        self.last_twist = Twist()
        # Command staleness is measured on the same clock the robot moves
        # on. A wall-clock threshold against a simulated-time publisher expires
        # once the real-time factor falls below `publish period / timeout`,
        # zeroing a live command mid-maneuver.
        self.last_command_time: float | None = None
        self.pod_publishers = {
            pod: self.create_publisher(Twist, f"/model/{pod}/cmd_vel", 10)
            for pod, value in self.catalog["inventory"].items()
            if value["type"] == "steer_drive_pod"
        }
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            MorphologyState, "morphology_state", self._on_morphology, qos
        )
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self._on_twist, 10
        )
        self.create_service(SetBool, "set_assembly_drive_enabled", self._set_enabled)
        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.create_timer(period, self._publish)

    def _on_morphology(self, message: MorphologyState) -> None:
        if message.morphology_id in self.catalog["morphologies"]:
            self.morphology = message.morphology_id

    def _on_twist(self, message: Twist) -> None:
        self.last_twist = message
        self.last_command_time = self._now_s()

    def _set_enabled(self, request, response):
        self.enabled = bool(request.data)
        if not self.enabled:
            self._stop_all()
        response.success = True
        response.message = (
            "assembly drive enabled" if self.enabled else "assembly drive inhibited"
        )
        return response

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish(self) -> None:
        if not self.enabled or not self.morphology:
            return
        timeout = float(self.get_parameter("command_timeout").value)
        command = (
            self.last_twist
            if (self.last_command_time is not None
                and self._now_s() - self.last_command_time <= timeout)
            else Twist()
        )
        morphology = self.catalog["morphologies"][self.morphology]
        limits = morphology["limits"]
        command.linear.x = max(
            -float(limits["linear"]),
            min(float(limits["linear"]), command.linear.x),
        )
        lateral = float(limits.get("lateral", 0.0))
        command.linear.y = (
            max(-lateral, min(lateral, command.linear.y)) if lateral else 0.0
        )
        command.angular.z = max(
            -float(limits["angular"]),
            min(float(limits["angular"]), command.angular.z),
        )
        max_speed = float(
            self.catalog["inventory"]["pod_0"].get("max_drive_speed", 1.2)
        )
        body_twist = BodyTwist(command.linear.x, command.linear.y, command.angular.z)
        commands = distribute_twist(
            body_twist, morphology["pods"], max_speed,
            float(morphology.get("yaw_effort_scale", 1.0)))
        for pod, value in commands.items():
            output = Twist()
            output.linear.x = value.linear
            output.angular.z = value.angular
            self.pod_publishers[pod].publish(output)

    def _stop_all(self) -> None:
        for publisher in self.pod_publishers.values():
            publisher.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AssemblyDriveAdapter()
    try:
        rclpy.spin(node)
    finally:
        node._stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
