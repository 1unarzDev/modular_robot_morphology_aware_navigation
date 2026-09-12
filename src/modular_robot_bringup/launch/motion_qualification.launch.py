import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory("modular_robot_sim")
    world = LaunchConfiguration("world")
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value=os.path.join(sim, "worlds", "indoor_doorway.sdf")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, "launch", "simulation.launch.py")),
            launch_arguments={"world": world}.items()),
        Node(package="morphology_manager", executable="morphology_manager",
             output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="modular_robot_bringup", executable="assembly_drive_adapter",
             output="screen", parameters=[{"use_sim_time": True}]),
    ])
