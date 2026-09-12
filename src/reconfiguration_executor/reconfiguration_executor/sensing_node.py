from __future__ import annotations

from math import atan2, cos, sin

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from modular_robot_msgs.msg import ConnectorState, MorphologyState, RelativePoseEstimate
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import LogicalCameraImage

from .control import Pose2, compose_pose
from .sensing import RelativePoseEstimator, RelativePoseObservation, anchored_odometry


def yaw_from_quaternion(quaternion) -> float:
    return atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class SensorRelativePoseNode(Node):
    """Fuse wheel-odometry and external fiducial observations per pod."""

    def __init__(self) -> None:
        super().__init__("pod_relative_pose_estimator")
        self.declare_parameter("stale_after_s", 0.5)
        self.declare_parameter("wheel_position_variance", 0.0025)
        self.declare_parameter("wheel_yaw_variance", 0.0076)
        default_catalog = get_package_share_directory("modular_robot_description") + "/config/morphologies.yaml"
        self.declare_parameter("catalog", default_catalog)
        with open(self.get_parameter("catalog").value, encoding="utf-8") as stream:
            self.catalog = yaml.safe_load(stream)
        self.estimator = RelativePoseEstimator(float(self.get_parameter("stale_after_s").value))
        self.morphology_id = "compact_diff"
        self.execution_state = MorphologyState.READY
        self.raw_odometry: dict[str, Pose2] = {}
        self.baselines: dict[str, Pose2] = {}
        self.anchors: dict[str, Pose2] = {}
        self.publisher = self.create_publisher(RelativePoseEstimate, "relative_pose_estimate", 20)
        self.create_subscription(MorphologyState, "morphology_state", self._on_morphology, 20)
        self.create_subscription(
            ConnectorState, "connector_state", self._on_connector_state, 20)
        for pod_id in [f"pod_{index}" for index in range(6)]:
            self.create_subscription(
                Odometry, f"/pods/{pod_id}/odometry",
                lambda message, pod=pod_id: self._on_pod(pod, message), 20,
            )
            self.create_subscription(
                PoseWithCovarianceStamped, f"/fiducials/{pod_id}/pose",
                lambda message, pod=pod_id: self._on_fiducial(pod, message), 20,
            )
        camera_mounts = {
            "front": Pose2(0.0, 0.0, 0.0),
            "left": Pose2(0.0, 0.0, 1.5707963),
            "rear": Pose2(0.0, 0.0, 3.1415927),
            "right": Pose2(0.0, 0.0, -1.5707963),
        }
        for name, mount in camera_mounts.items():
            self.create_subscription(
                LogicalCameraImage, f"/connector_sensor/{name}",
                lambda message, sensor=name, pose=mount: self._on_logical_camera(sensor, pose, message), 20,
            )

    def _on_morphology(self, message: MorphologyState) -> None:
        entering_transition = (
            message.execution_state == MorphologyState.TRANSITIONING
            and self.execution_state != MorphologyState.TRANSITIONING
        )
        self.morphology_id = message.morphology_id
        self.execution_state = message.execution_state
        if entering_transition:
            self.baselines = dict(self.raw_odometry)
            self.anchors = {
                pod: Pose2(*(float(value) for value in pose))
                for pod, pose in self.catalog["morphologies"][self.morphology_id]["pods"].items()
            }

    def _on_pod(self, pod_id: str, message: Odometry) -> None:
        pose = message.pose.pose
        raw = Pose2(pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        self.raw_odometry[pod_id] = raw
        nominal = self.catalog["morphologies"][self.morphology_id]["pods"][pod_id]
        anchor = self.anchors.get(pod_id, Pose2(*(float(value) for value in nominal)))
        baseline = self.baselines.setdefault(pod_id, raw)
        pod = anchored_odometry(anchor, baseline, raw)
        now = self.get_clock().now().nanoseconds / 1e9
        if not self._update(RelativePoseObservation(
            pod_id, "wheel_odometry", now, pod,
            float(self.get_parameter("wheel_position_variance").value),
            float(self.get_parameter("wheel_position_variance").value),
            float(self.get_parameter("wheel_yaw_variance").value), False,
        )):
            return
        self._publish(pod_id, now)

    def _on_connector_state(self, message: ConnectorState) -> None:
        if message.state != ConnectorState.LATCHED or "/" not in message.parent_port:
            return
        morphology, pod = message.parent_port.split("/", 1)
        if (pod != message.pod_id or morphology not in self.catalog["morphologies"]
                or pod not in self.raw_odometry):
            return
        self.baselines[pod] = self.raw_odometry[pod]
        self.anchors[pod] = Pose2(*(
            float(value)
            for value in self.catalog["morphologies"][morphology]["pods"][pod]
        ))
        self.estimator.reset_pod(pod)

    def _on_fiducial(self, pod_id: str, message: PoseWithCovarianceStamped) -> None:
        pose, covariance = message.pose.pose, message.pose.covariance
        now = self.get_clock().now().nanoseconds / 1e9
        if not self._update(RelativePoseObservation(
            pod_id, "fiducial", now,
            Pose2(pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)),
            max(1e-8, covariance[0]), max(1e-8, covariance[7]),
            max(1e-8, covariance[35]), True,
        )):
            return
        self._publish(pod_id, now)

    def _on_logical_camera(
        self, sensor_name: str, mount: Pose2, message: LogicalCameraImage
    ) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        for model in message.model:
            pod_id = model.name.split("::", 1)[0]
            if pod_id not in self.catalog["inventory"] or not pod_id.startswith("pod_"):
                continue
            pose = model.pose
            in_sensor = Pose2(
                pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)
            )
            in_core = compose_pose(mount, [in_sensor.x, in_sensor.y, in_sensor.yaw])
            range_squared = in_sensor.x * in_sensor.x + in_sensor.y * in_sensor.y
            # Gazebo's logical camera reports an exact relative model pose. Use
            # covariance consistent with that measurement so topology-anchored
            # wheel odometry cannot bias final latch alignment. Sensing trials
            # must inject pose noise and change this covariance together.
            position_variance = 1e-6 + 1e-5 * range_squared
            if not self._update(RelativePoseObservation(
                pod_id, "fiducial", now, in_core, position_variance,
                position_variance, 1e-6 + 2e-5 * range_squared, True,
            )):
                continue
            self._publish(pod_id, now)

    def _update(self, observation: RelativePoseObservation) -> bool:
        try:
            self.estimator.update(observation)
            return True
        except ValueError as exc:
            if "stale or out-of-order" not in str(exc):
                raise
            return False

    def _publish(self, pod_id: str, now: float) -> None:
        estimate = self.estimator.estimate(pod_id, now)
        message = RelativePoseEstimate()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "core/base_link"
        message.pod_id = pod_id
        message.pose.pose.position.x = estimate.pose.x
        message.pose.pose.position.y = estimate.pose.y
        message.pose.pose.orientation.z = sin(estimate.pose.yaw / 2)
        message.pose.pose.orientation.w = cos(estimate.pose.yaw / 2)
        message.pose.covariance[0] = estimate.variance_x
        message.pose.covariance[7] = estimate.variance_y
        message.pose.covariance[35] = estimate.variance_yaw
        message.uncertainty_class = int(estimate.uncertainty_class)
        message.connector_visible = estimate.visible
        message.sources = list(estimate.sources)
        message.sensing_revision = estimate.sensing_revision
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorRelativePoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
