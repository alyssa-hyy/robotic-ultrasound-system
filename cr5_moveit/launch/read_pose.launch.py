"""Read CR5 link pose using the same robot model as the MoveIt launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "cr5_robot", package_name="cr5_moveit"
    ).to_moveit_configs()

    return LaunchDescription([
        DeclareLaunchArgument("planning_group", default_value="cr5_group"),
        DeclareLaunchArgument("end_effector_link", default_value="probe_tcp"),
        DeclareLaunchArgument("state_timeout", default_value="10.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="cr5_moveit",
            executable="cr5_read_pose",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {
                    "planning_group": ParameterValue(
                        LaunchConfiguration("planning_group"), value_type=str),
                    "end_effector_link": ParameterValue(
                        LaunchConfiguration("end_effector_link"), value_type=str),
                    "state_timeout": ParameterValue(
                        LaunchConfiguration("state_timeout"), value_type=float),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
        ),
    ])
