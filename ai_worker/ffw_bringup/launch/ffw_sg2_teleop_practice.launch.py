#!/usr/bin/env python3
"""One-command SG2 teleoperation practice bringup.

Starts follower control, low-latency cameras/feedback, and LG2 leader.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_launch_dir = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')
    teleop_launch_dir = os.path.join(get_package_share_directory('ffw_teleop'), 'launch')

    leader_controller_config = LaunchConfiguration('leader_controller_config')
    init_position = LaunchConfiguration('init_position')
    launch_lidar = LaunchConfiguration('launch_lidar')
    start_feedback = LaunchConfiguration('start_feedback')
    start_leader = LaunchConfiguration('start_leader')

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'ffw_sg2_follower_ai.launch.py')),
        launch_arguments={
            'launch_cameras': 'false',
            'launch_lidar': launch_lidar,
            'init_position': init_position,
            'start_rviz': 'false',
        }.items(),
    )

    feedback = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_launch_dir, 'teleop_wrist_depth.launch.py')),
        launch_arguments={
            'start_zed': LaunchConfiguration('start_zed'),
            'start_wrist_cameras': LaunchConfiguration('start_wrist_cameras'),
            'start_left_wrist': LaunchConfiguration('start_left_wrist'),
            'start_right_wrist': LaunchConfiguration('start_right_wrist'),
            'start_overlay': LaunchConfiguration('start_right_overlay'),
            'start_left_overlay': LaunchConfiguration('start_left_overlay'),
            'overlay_fps': LaunchConfiguration('overlay_fps'),
            'publish_raw_overlay': LaunchConfiguration('publish_raw_overlay'),
            'record_practice_events': LaunchConfiguration('record_practice_events'),
            'practice_event_log_path': LaunchConfiguration('practice_event_log_path'),
            'table_reference_enabled': LaunchConfiguration('table_reference_enabled'),
            'table_x_m': LaunchConfiguration('table_x_m'),
            'table_y_m': LaunchConfiguration('table_y_m'),
            'table_yaw_deg': LaunchConfiguration('table_yaw_deg'),
        }.items(),
        condition=IfCondition(start_feedback),
    )

    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'ffw_lg2_leader_ai.launch.py')),
        launch_arguments={
            'leader_controller_config': leader_controller_config,
        }.items(),
        condition=IfCondition(start_leader),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'leader_controller_config',
            default_value='ffw_lg2_leader_ai_hardware_controller_no_head.yaml'),
        DeclareLaunchArgument('init_position', default_value='true'),
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('start_feedback', default_value='true'),
        DeclareLaunchArgument('start_leader', default_value='true'),
        DeclareLaunchArgument('start_zed', default_value='true'),
        DeclareLaunchArgument('start_wrist_cameras', default_value='true'),
        DeclareLaunchArgument('start_left_wrist', default_value='true'),
        DeclareLaunchArgument('start_right_wrist', default_value='true'),
        DeclareLaunchArgument('start_left_overlay', default_value='true'),
        DeclareLaunchArgument('start_right_overlay', default_value='true'),
        DeclareLaunchArgument('overlay_fps', default_value='10.0'),
        DeclareLaunchArgument('publish_raw_overlay', default_value='false'),
        DeclareLaunchArgument('record_practice_events', default_value='true'),
        DeclareLaunchArgument(
            'practice_event_log_path', default_value='~/teleop_practice_events.jsonl'),
        DeclareLaunchArgument('table_reference_enabled', default_value='false'),
        DeclareLaunchArgument('table_x_m', default_value='0.0'),
        DeclareLaunchArgument('table_y_m', default_value='0.0'),
        DeclareLaunchArgument('table_yaw_deg', default_value='0.0'),
        follower,
        TimerAction(period=25.0, actions=[feedback]),
        TimerAction(period=30.0, actions=[leader]),
    ])
