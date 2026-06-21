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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    leader_controller_config = LaunchConfiguration('leader_controller_config')
    leader_left_port = LaunchConfiguration('leader_left_port')
    leader_right_port = LaunchConfiguration('leader_right_port')
    start_rviz = LaunchConfiguration('start_rviz')
    start_leader = LaunchConfiguration('start_leader')
    start_mission_control = LaunchConfiguration('start_mission_control')
    start_operator_layout = LaunchConfiguration('start_operator_layout')
    rviz_config = LaunchConfiguration('rviz_config')
    mission_profiles_config = LaunchConfiguration('mission_profiles_config')
    operator_screen_layout_config = LaunchConfiguration('operator_screen_layout_config')
    operator_layout_action = LaunchConfiguration('operator_layout_action')
    operator_layout_store_path = LaunchConfiguration('operator_layout_store_path')

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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='teleop_operator_rviz',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    mission_control = Node(
        package='ffw_teleop',
        executable='mission_mode_manager',
        name='mission_mode_manager',
        output='screen',
        parameters=[{
            'profiles_config': mission_profiles_config,
        }],
        condition=IfCondition(start_mission_control),
    )

    operator_layout = Node(
        package='ffw_teleop',
        executable='operator_layout_manager',
        name='operator_layout_manager',
        output='screen',
        parameters=[{
            'layout_config': operator_screen_layout_config,
            'action': operator_layout_action,
            'layout_store_path': operator_layout_store_path,
            'initial_delay_s': 3.0,
            'retry_count': 24,
            'retry_interval_s': 0.5,
        }],
        condition=IfCondition(start_operator_layout),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'leader_controller_config',
            default_value='ffw_lg2_leader_ai_hardware_controller.yaml'),
        DeclareLaunchArgument('leader_left_port', default_value='/dev/left_leader'),
        DeclareLaunchArgument('leader_right_port', default_value='/dev/right_leader'),
        DeclareLaunchArgument('start_leader', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_mission_control', default_value='true'),
        DeclareLaunchArgument('start_operator_layout', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'rviz',
                'teleop_operator.rviz',
            ])),
        DeclareLaunchArgument(
            'mission_profiles_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'config',
                'mission_profiles.yaml',
            ])),
        DeclareLaunchArgument(
            'operator_screen_layout_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'config',
                'operator_screen_layout.yaml',
            ])),
        DeclareLaunchArgument('operator_layout_action', default_value='server'),
        DeclareLaunchArgument(
            'operator_layout_store_path',
            default_value='~/.config/ffw_teleop/operator_screen_layout.json'),
        leader,
        rviz,
        mission_control,
        operator_layout,
    ])
