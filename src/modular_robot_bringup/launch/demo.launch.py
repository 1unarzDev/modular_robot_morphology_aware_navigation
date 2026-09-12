import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("modular_robot_bringup")
    sim = get_package_share_directory("modular_robot_sim")
    nav2 = get_package_share_directory("nav2_bringup")
    slam = get_package_share_directory("slam_toolbox")
    world = LaunchConfiguration("world")
    map_file = LaunchConfiguration("map")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    has_map = PythonExpression(["'", map_file, "' != ''"])
    localization_params = os.path.join(bringup, "config", "localization.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value=os.path.join(sim, "worlds", "indoor_doorway.sdf")),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, "launch", "simulation.launch.py")),
            launch_arguments={"world": world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam, "launch", "online_async_launch.py")),
            condition=UnlessCondition(has_map),
            launch_arguments={"slam_params_file": os.path.join(bringup, "config", "slam.yaml"), "use_sim_time": "true"}.items(),
        ),
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", condition=IfCondition(has_map),
             parameters=[localization_params,
                         {"use_sim_time": True, "yaml_filename": map_file}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
             condition=IfCondition(has_map),
             parameters=[localization_params, {
                 "use_sim_time": True, "set_initial_pose": True,
                 "initial_pose.x": initial_x, "initial_pose.y": initial_y,
                 "initial_pose.yaw": 0.0,
             }]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             condition=IfCondition(has_map), parameters=[{
                 "use_sim_time": True, "autostart": True,
                 "node_names": ["map_server", "amcl"],
             }]),
        TimerAction(period=3.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "navigation_launch.py")),
            launch_arguments={"params_file": os.path.join(bringup, "config", "nav2.yaml"), "use_sim_time": "true"}.items(),
        )]),
        Node(package="morphology_manager", executable="morphology_manager", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="modular_robot_bringup", executable="assembly_drive_adapter", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="lidar_static_tf",
             arguments=["--x", "0", "--y", "0", "--z", "0.12", "--frame-id", "core/base_link", "--child-frame-id", "core/lidar_link/lidar"]),
        Node(package="morphology_planner", executable="planner_server", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="reconfiguration_executor", executable="reconfiguration_executor", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="reconfiguration_executor", executable="pod_relative_pose_estimator", output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="modular_robot_bringup", executable="hybrid_navigator", output="screen", parameters=[{"use_sim_time": True}]),
    ])
