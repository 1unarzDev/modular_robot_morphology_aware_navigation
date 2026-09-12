from __future__ import annotations

import argparse
import json
from math import atan2, cos, sin
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


class MotionQualifier(Node):
    def __init__(self) -> None:
        super().__init__("assembly_motion_qualifier")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sim_time: float | None = None
        self.odometry: Odometry | None = None
        self.samples: list[dict] = []
        self.last_sample_time: float | None = None
        self.pod_commands = {f"pod_{index}": 0.0 for index in range(6)}
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odometry, qos_profile_sensor_data)
        for pod in self.pod_commands:
            self.create_subscription(
                Twist, f"/model/{pod}/cmd_vel",
                lambda message, pod=pod: self._pod(pod, message), 10)

    def _on_clock(self, message: Clock) -> None:
        self.sim_time = message.clock.sec + message.clock.nanosec * 1e-9

    def _on_odometry(self, message: Odometry) -> None:
        self.odometry = message

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


def run(output: Path, watchdog_s: float = 90.0) -> dict:
    rclpy.init()
    node = MotionQualifier()
    started = time.monotonic()
    try:
        while (node.sim_time is None or node.odometry is None) and time.monotonic() - started < watchdog_s:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.sim_time is None or node.odometry is None:
            raise RuntimeError("clock/odometry readiness timeout")
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
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        return result
    finally:
        node.publish(0.0, 0.0, "stop")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def summarize(samples: list[dict]) -> dict:
    summaries = {}
    for stage in ("positive_yaw", "negative_yaw", "straight"):
        values = [value for value in samples if value["stage"] == stage]
        if not values:
            raise RuntimeError(f"no samples for {stage}")
        summaries[stage] = {
            "duration_s": values[-1]["time_s"] - values[0]["time_s"],
            "delta_x": values[-1]["x"] - values[0]["x"],
            "delta_y": values[-1]["y"] - values[0]["y"],
            "delta_yaw": _angle_difference(values[-1]["yaw"], values[0]["yaw"]),
            "peak_absolute_pod_commands": {
                pod: max(abs(value["pod_commands"][pod]) for value in values)
                for pod in values[0]["pod_commands"]
            },
        }
    positive_ok = summaries["positive_yaw"]["delta_yaw"] > 0.1
    negative_ok = summaries["negative_yaw"]["delta_yaw"] < -0.1
    straight_ok = summaries["straight"]["delta_x"] > 0.1
    return {
        "schema_version": 1, "purpose": "engineering_qualification",
        "rep_103_signs_pass": positive_ok and negative_ok,
        "straight_motion_pass": straight_ok,
        "stages": summaries, "sample_count": len(samples), "samples": samples,
    }


def _angle_difference(target: float, source: float) -> float:
    return atan2(sin(target - source), cos(target - source))


def main() -> None:
    parser = argparse.ArgumentParser(description="Qualify assembled motion sign conventions")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--watchdog", type=float, default=90.0)
    args = parser.parse_args()
    print(json.dumps(run(args.output, args.watchdog), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
