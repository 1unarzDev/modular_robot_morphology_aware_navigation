import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("modular_robot_bringup")
    sim = get_package_share_directory("modular_robot_sim")
    nav2 = get_package_share_directory("nav2_bringup")
    slam = get_package_share_directory("slam_toolbox")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim, "launch", "simulation.launch.py"))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam, "launch", "online_async_launch.py")),
            launch_arguments={"slam_params_file": os.path.join(bringup, "config", "slam.yaml"), "use_sim_time": "true"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "navigation_launch.py")),
            launch_arguments={"params_file": os.path.join(bringup, "config", "nav2.yaml"), "use_sim_time": "true"}.items(),
        ),
        Node(package="morphology_manager", executable="morphology_manager", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="modular_robot_bringup", executable="assembly_drive_adapter", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="lidar_static_tf",
             arguments=["--x", "0", "--y", "0", "--z", "0.12", "--frame-id", "core/base_link", "--child-frame-id", "core/lidar_link/lidar"]),
        Node(package="morphology_planner", executable="planner_server", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="reconfiguration_executor", executable="reconfiguration_executor", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="reconfiguration_executor", executable="pod_relative_pose_estimator", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="modular_robot_bringup", executable="hybrid_navigator", output="screen", parameters=[{"use_sim_time": True}]),
    ])
