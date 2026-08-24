#!/usr/bin/env python3
"""Display a Booster K1 model with joint controls and RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "booster_k1_description"
MODEL_FILES = {
    "standard": "k1_22dof.urdf",
    "wheelfoot": "k1_wheelfoot.urdf",
}


def _launch_setup(context: LaunchContext) -> list[Node]:
    model = LaunchConfiguration("model").perform(context)
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    model_path = package_share / "urdf" / MODEL_FILES[model]
    robot_description = model_path.read_text(encoding="utf-8")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"),
        value_type=bool,
    )
    description_parameters = {
        "robot_description": robot_description,
        "use_sim_time": use_sim_time,
    }

    return [
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=UnlessCondition(LaunchConfiguration("use_gui")),
            parameters=[description_parameters],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("use_gui")),
            parameters=[description_parameters],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[description_parameters],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(package_share / "rviz" / "k1.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="standard",
                choices=tuple(MODEL_FILES),
                description="Robot model variant to display.",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                choices=("true", "false"),
                description="Use the graphical joint state publisher.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                choices=("true", "false"),
                description="Start RViz with the package configuration.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=("true", "false"),
                description="Use a simulation clock.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
