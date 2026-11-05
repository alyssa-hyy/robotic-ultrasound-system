"""Mark the startup TCP scan center and an optional XY reference circle."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = MoveItConfigsBuilder(
        "cr5_robot", package_name="cr5_moveit"
    ).to_moveit_configs()
    return LaunchDescription([
        DeclareLaunchArgument("radius", default_value="0.05"),
        DeclareLaunchArgument("show_circle", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="cr5_moveit",
            executable="cr5_scan_center_marker",
            output="screen",
            parameters=[
                config.robot_description,
                config.robot_description_semantic,
                config.robot_description_kinematics,
                {
                    "radius": ParameterValue(
                        LaunchConfiguration("radius"), value_type=float),
                    "show_circle": ParameterValue(
                        LaunchConfiguration("show_circle"), value_type=bool),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
        ),
    ])
