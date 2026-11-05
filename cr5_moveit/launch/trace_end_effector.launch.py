"""Observe the CR5 probe path from joint state feedback, without moving it."""

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
        DeclareLaunchArgument("end_effector_link", default_value="probe_tcp"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="cr5_moveit",
            executable="cr5_trace_end_effector",
            output="screen",
            parameters=[
                config.robot_description,
                config.robot_description_semantic,
                config.robot_description_kinematics,
                {
                    "end_effector_link": ParameterValue(
                        LaunchConfiguration("end_effector_link"), value_type=str),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
        ),
    ])
