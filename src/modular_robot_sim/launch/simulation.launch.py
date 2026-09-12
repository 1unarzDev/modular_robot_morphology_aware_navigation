import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("modular_robot_sim")
    gz_share = get_package_share_directory("ros_gz_sim")
    plugin_prefix = get_package_prefix("modular_robot_gz_plugins")
    resource_path = os.path.join(share, "models")
    world = LaunchConfiguration("world")
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value=os.path.join(share, "worlds", "indoor_doorway.sdf")),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", os.path.join(plugin_prefix, "lib")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_share, "launch", "gz_sim.launch.py")),
            # DART supports detachable fixed joints and differential pod yaw.
            # Pod internals have unique names because DART merges redocked
            # children into the core skeleton.
            launch_arguments={"gz_args": ["-r -s ", world]}.items(),
        ),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
             parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml")}], output="screen"),
    ])
