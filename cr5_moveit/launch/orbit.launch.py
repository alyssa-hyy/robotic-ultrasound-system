"""Orbit with optional radial orientation, FK coverage validation and return to startup pose."""

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
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("follow_radial", default_value="true"),
        DeclareLaunchArgument("return_center", default_value="true"),
        DeclareLaunchArgument("speed_scale", default_value="0.1"),
        DeclareLaunchArgument("radius", default_value="0.05"),
        DeclareLaunchArgument("clockwise", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="cr5_moveit",
            executable="cr5_orbit",
            output="screen",
            parameters=[
                config.robot_description,
                config.robot_description_semantic,
                config.robot_description_kinematics,
                config.joint_limits,
                {
                    "follow_radial": ParameterValue(
                        LaunchConfiguration("follow_radial"), value_type=bool),
                    "return_center": ParameterValue(
                        LaunchConfiguration("return_center"), value_type=bool),
                    "speed_scale": ParameterValue(
                        LaunchConfiguration("speed_scale"), value_type=float),
                    "radius": ParameterValue(
                        LaunchConfiguration("radius"), value_type=float),
                    "clockwise": ParameterValue(
                        LaunchConfiguration("clockwise"), value_type=bool),
                    "execute": ParameterValue(
                        LaunchConfiguration("execute"), value_type=bool),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
        ),
    ])
