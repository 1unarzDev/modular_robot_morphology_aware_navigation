import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("modular_robot_sim")
    gz_share = get_package_share_directory("ros_gz_sim")
    plugin_prefix = get_package_prefix("modular_robot_gz_plugins")
    resource_path = os.path.join(share, "models")
    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", os.path.join(plugin_prefix, "lib")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_share, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": f"-r -s --physics-engine gz-physics-bullet-featherstone-plugin {share}/worlds/indoor_doorway.sdf"}.items(),
        ),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
             parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml")}], output="screen"),
    ])
