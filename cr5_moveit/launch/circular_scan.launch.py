"""Scan 12 stations with a radial probe long edge and local transverse Y swings."""

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
        DeclareLaunchArgument("ik_attempts", default_value="12"),
        DeclareLaunchArgument("clockwise", default_value="false"),
        DeclareLaunchArgument("dwell_seconds", default_value="2.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="cr5_moveit",
            executable="cr5_circular_scan",
            output="screen",
            parameters=[
                config.robot_description,
                config.robot_description_semantic,
                config.robot_description_kinematics,
                config.joint_limits,
                {
                    "ik_attempts": ParameterValue(
                        LaunchConfiguration("ik_attempts"), value_type=int),
                    "clockwise": ParameterValue(
                        LaunchConfiguration("clockwise"), value_type=bool),
                    "dwell_seconds": ParameterValue(
                        LaunchConfiguration("dwell_seconds"), value_type=float),
                    "execute": ParameterValue(
                        LaunchConfiguration("execute"), value_type=bool),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
        ),
    ])
