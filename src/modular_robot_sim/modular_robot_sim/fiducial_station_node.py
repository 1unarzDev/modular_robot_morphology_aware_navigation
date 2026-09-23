"""Workspace fiducial station (ADR 0006).

Precision docking depends on an external observer, so this node synthesizes
what that observer delivers. A pod is station-observed when a declared station
has an unoccluded sight line to it; the node then publishes that pod's pose
relative to the core on the `/fiducials/<pod>/pose` hook the estimator already
carries. A pod standing in a station's shadow is simply not published, its
station observation goes stale, and the estimator reports the connector
unobservable -- which is what makes observability a property of *where* the
robot chose to transition.

This is the simulator's sensor model, so it reads simulator state, exactly as
the lidar and the logical cameras do. It lives outside the five autonomy
packages the ground-truth guard scans, and nothing it reads reaches autonomy
except as this synthesized observation.
"""

from __future__ import annotations

import json
from math import cos, sin

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from morphology_planner.observability import (
    load_fiducial_stations, observed_by_any, precise_variances,
)
from morphology_planner.transition_validation import load_environment_boxes

POD_IDS = tuple(f"pod_{index}" for index in range(6))
POSE_KEYS = ("world_x", "world_y", "world_z", "world_yaw")
CORE_KEYS = ("core_world_x", "core_world_y", "core_world_yaw")


class FiducialStationNode(Node):
    def __init__(self) -> None:
        super().__init__("fiducial_station")
        self.declare_parameter("scenario_manifest", "")
        manifest = str(self.get_parameter("scenario_manifest").value)
        self.stations = load_fiducial_stations(manifest)
        self.obstacles = load_environment_boxes(manifest)
        self.publishers_by_pod = {
            pod: self.create_publisher(
                PoseWithCovarianceStamped, f"/fiducials/{pod}/pose", 20)
            for pod in POD_IDS
        }
        for pod in POD_IDS:
            self.create_subscription(
                String, f"/evaluator/pods/{pod}/drive_diagnostics",
                lambda message, observed=pod: self._on_diagnostics(observed, message),
                20,
            )
        if not self.stations:
            self.get_logger().info(
                "no fiducial station declared; connector observability is not "
                "station-gated in this world")
        else:
            self.get_logger().info(
                f"{len(self.stations)} fiducial station(s), "
                f"{len(self.obstacles)} declared obstacle(s)")

    def _on_diagnostics(self, pod: str, message: String) -> None:
        if not self.stations:
            return
        try:
            sample = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not all(key in sample for key in POSE_KEYS):
            return
        if not all(key in sample for key in CORE_KEYS):
            return
        point = (float(sample["world_x"]), float(sample["world_y"]),
                 float(sample["world_z"]))
        station = observed_by_any(self.stations, point, self.obstacles)
        if station is None:
            # In shadow: publish nothing, let the observation go stale.
            return
        self._publish(pod, sample, point, *precise_variances(station.range_to(point)))

    def _publish(self, pod: str, sample: dict, point: tuple[float, float, float],
                 position_variance: float, yaw_variance: float) -> None:
        core_x, core_y, core_yaw = (float(sample[key]) for key in CORE_KEYS)
        dx, dy = point[0] - core_x, point[1] - core_y
        # The estimator's fiducial hook takes the pod pose in the core frame,
        # the same convention the onboard connector cameras report.
        relative_x = cos(-core_yaw) * dx - sin(-core_yaw) * dy
        relative_y = sin(-core_yaw) * dx + cos(-core_yaw) * dy
        relative_yaw = float(sample["world_yaw"]) - core_yaw

        output = PoseWithCovarianceStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "core/base_link"
        output.pose.pose.position.x = relative_x
        output.pose.pose.position.y = relative_y
        output.pose.pose.orientation.z = sin(relative_yaw / 2.0)
        output.pose.pose.orientation.w = cos(relative_yaw / 2.0)
        # The estimator reads the trace back from these three entries.
        output.pose.covariance[0] = position_variance
        output.pose.covariance[7] = position_variance
        output.pose.covariance[35] = yaw_variance
        self.publishers_by_pod[pod].publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FiducialStationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
