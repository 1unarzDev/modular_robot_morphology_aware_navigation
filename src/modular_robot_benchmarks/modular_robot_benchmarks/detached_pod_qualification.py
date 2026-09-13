from __future__ import annotations

import argparse
import json
from math import atan2, cos, hypot, sin
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def _angle_delta(end: float, start: float) -> float:
    return atan2(sin(end - start), cos(end - start))


class DetachedPodQualifier(Node):
    def __init__(self) -> None:
        super().__init__("detached_pod_qualifier")
        self.publisher = self.create_publisher(Twist, "/model/pod_0/cmd_vel", 10)
        self.sim_time: float | None = None
        self.latest: dict | None = None
        self.samples: list[dict] = []
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.create_subscription(
            String, "/evaluator/pods/pod_0/drive_diagnostics", self._sample, 50)

    def _on_clock(self, message: Clock) -> None:
        self.sim_time = message.clock.sec + message.clock.nanosec * 1e-9

    def _sample(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except ValueError:
            return
        if value.get("pod") == "pod_0":
            self.latest = value
            self.samples.append(value)

    def command(self, linear: float, angular: float) -> None:
        value = Twist()
        value.linear.x, value.angular.z = linear, angular
        self.publisher.publish(value)


def _stage(samples: list[dict], name: str, start: dict, end: dict) -> dict:
    dx, dy = end["world_x"] - start["world_x"], end["world_y"] - start["world_y"]
    yaw = start["world_yaw"]
    stage_samples = [value for value in samples if value.get("stage") == name]
    tracking = [
        abs(value[side + "_command_radps"] - value[side + "_measured_radps"])
        for value in stage_samples for side in ("left", "right")
    ]
    return {
        "forward_m": cos(yaw) * dx + sin(yaw) * dy,
        "lateral_m": -sin(yaw) * dx + cos(yaw) * dy,
        "translation_m": hypot(dx, dy),
        "yaw_rad": _angle_delta(end["world_yaw"], yaw),
        "wheel_tracking_mae_radps": sum(tracking) / len(tracking) if tracking else None,
        "sample_count": len(stage_samples),
    }


def run(output: Path, watchdog_s: float = 60.0) -> dict:
    rclpy.init()
    node = DetachedPodQualifier()
    wall_start = time.monotonic()
    stages = (
        ("positive_yaw", 0.0, 0.6, 3.0),
        ("negative_yaw", 0.0, -0.6, 3.0),
        ("forward", 0.25, 0.0, 3.0),
        ("reverse", -0.25, 0.0, 3.0),
    )
    results = {}
    try:
        while (node.sim_time is None or node.latest is None) and time.monotonic() - wall_start < watchdog_s:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.sim_time is None or node.latest is None:
            raise RuntimeError("detached pod diagnostic readiness timeout")
        for name, linear, angular, duration in stages:
            node.command(0.0, 0.0)
            settle_start = node.sim_time
            while node.sim_time - settle_start < 1.0:
                rclpy.spin_once(node, timeout_sec=0.02)
            start = dict(node.latest)
            stage_start = node.sim_time
            first_index = len(node.samples)
            while node.sim_time - stage_start < duration:
                node.command(linear, angular)
                rclpy.spin_once(node, timeout_sec=0.02)
                if time.monotonic() - wall_start > watchdog_s:
                    raise RuntimeError("detached pod qualification watchdog exceeded")
            node.command(0.0, 0.0)
            end = dict(node.latest)
            for value in node.samples[first_index:]:
                value["stage"] = name
            results[name] = _stage(node.samples, name, start, end)
        passed = (
            results["positive_yaw"]["yaw_rad"] > 0.4
            and results["negative_yaw"]["yaw_rad"] < -0.4
            and results["forward"]["forward_m"] > 0.5
            and results["reverse"]["forward_m"] < -0.5
            and max(abs(results[name]["lateral_m"]) for name in ("forward", "reverse")) < 0.12
        )
        result = {"schema_version": 1, "purpose": "engineering_detached_pod_qualification", "passed": passed, "stages": results}
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        return result
    finally:
        node.command(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="Qualify one independently mobile pod")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--watchdog", type=float, default=60.0)
    args = parser.parse_args()
    print(json.dumps(run(args.output, args.watchdog), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
