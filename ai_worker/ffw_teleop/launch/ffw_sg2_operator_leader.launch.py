#!/usr/bin/env python3
"""Operator-side SG2 teleoperation launch.

Runs the RViz operator view and the LG2 leader controller on the main PC.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    leader_controller_config = LaunchConfiguration('leader_controller_config')
    leader_left_port = LaunchConfiguration('leader_left_port')
    leader_right_port = LaunchConfiguration('leader_right_port')
    start_rviz = LaunchConfiguration('start_rviz')
    start_leader = LaunchConfiguration('start_leader')

    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ffw_bringup'),
                'launch',
                'ffw_lg2_leader_ai.launch.py',
            ])),
        launch_arguments={
            'leader_controller_config': leader_controller_config,
            'leader_left_port': leader_left_port,
            'leader_right_port': leader_right_port,
        }.items(),
        condition=IfCondition(start_leader),
    )

    operator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'launch',
                'teleop_operator.launch.py',
            ])),
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'leader_controller_config',
            default_value='ffw_lg2_leader_ai_hardware_controller.yaml'),
        DeclareLaunchArgument('leader_left_port', default_value='/dev/left_leader'),
        DeclareLaunchArgument('leader_right_port', default_value='/dev/right_leader'),
        DeclareLaunchArgument('start_leader', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        leader,
        operator,
    ])
