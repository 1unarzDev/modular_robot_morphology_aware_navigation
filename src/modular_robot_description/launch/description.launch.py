from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("modular_robot_description"), "urdf", "modular_robot.urdf.xacro"]),
    ])
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": True}]),
    ])

