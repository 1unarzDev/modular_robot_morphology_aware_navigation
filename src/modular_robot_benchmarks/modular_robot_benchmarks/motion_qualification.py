from __future__ import annotations

import argparse
import json
from math import atan2
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from modular_robot_msgs.msg import MorphologyState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

from .motion_metrics import summarize


class MotionQualifier(Node):
    def __init__(self) -> None:
        super().__init__("assembly_motion_qualifier")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sim_time: float | None = None
        self.odometry: Odometry | None = None
        self.morphology: MorphologyState | None = None
        self.samples: list[dict] = []
        self.last_sample_time: float | None = None
        self.pod_commands = {f"pod_{index}": 0.0 for index in range(6)}
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(
            MorphologyState, "morphology_state", self._on_morphology, 10)
        for pod in self.pod_commands:
            self.create_subscription(
                Twist, f"/model/{pod}/cmd_vel",
                lambda message, pod=pod: self._pod(pod, message), 10)

    def _on_clock(self, message: Clock) -> None:
        self.sim_time = message.clock.sec + message.clock.nanosec * 1e-9

    def _on_odometry(self, message: Odometry) -> None:
        self.odometry = message

    def _on_morphology(self, message: MorphologyState) -> None:
        self.morphology = message

    def _pod(self, pod: str, message: Twist) -> None:
        self.pod_commands[pod] = float(message.linear.x)

    def publish(self, linear: float, angular: float, stage: str) -> None:
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.publisher.publish(command)
        if (self.odometry is not None and self.sim_time is not None and
                (self.last_sample_time is None or
                 self.sim_time - self.last_sample_time >= 0.05)):
            pose = self.odometry.pose.pose
            self.samples.append({
                "stage": stage, "time_s": self.sim_time,
                "x": float(pose.position.x), "y": float(pose.position.y),
                "yaw": yaw(pose.orientation), "linear_command": linear,
                "angular_command": angular, "pod_commands": dict(self.pod_commands),
            })
            self.last_sample_time = self.sim_time


def yaw(quaternion) -> float:
    return atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def run(output: Path, watchdog_s: float = 90.0,
        expected_morphology: str | None = None) -> dict:
    rclpy.init()
    node = MotionQualifier()
    started = time.monotonic()
    try:
        while (node.sim_time is None or node.odometry is None
               or node.morphology is None
               or (expected_morphology is not None
                   and node.morphology.morphology_id != expected_morphology)) \
                and time.monotonic() - started < watchdog_s:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.sim_time is None or node.odometry is None or node.morphology is None:
            raise RuntimeError("clock/odometry/morphology readiness timeout")
        if (expected_morphology is not None
                and node.morphology.morphology_id != expected_morphology):
            raise RuntimeError(
                f"expected morphology {expected_morphology}, observed "
                f"{node.morphology.morphology_id}")
        stages = (
            ("settle", 0.0, 0.0, 2.0),
            ("positive_yaw", 0.0, 0.35, 4.0),
            ("settle", 0.0, 0.0, 2.0),
            ("negative_yaw", 0.0, -0.35, 4.0),
            ("settle", 0.0, 0.0, 2.0),
            ("straight", 0.20, 0.0, 4.0),
            ("settle", 0.0, 0.0, 2.0),
        )
        for stage, linear, angular, duration in stages:
            stage_start = node.sim_time
            while node.sim_time - stage_start < duration:
                node.publish(linear, angular, stage)
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.monotonic() - started >= watchdog_s:
                    raise RuntimeError("motion qualification watchdog exceeded")
        result = summarize(node.samples)
        result["morphology"] = node.morphology.morphology_id
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        return result
    finally:
        node.publish(0.0, 0.0, "stop")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



def main() -> None:
    parser = argparse.ArgumentParser(description="Qualify assembled motion sign conventions")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--watchdog", type=float, default=90.0)
    parser.add_argument("--expected-morphology")
    args = parser.parse_args()
    print(json.dumps(run(
        args.output, args.watchdog, args.expected_morphology),
        indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
