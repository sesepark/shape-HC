#!/usr/bin/env python3
"""Legacy SG2 teleoperation practice bringup.

Starts follower control and low-latency cameras/feedback. The LG2 leader is
disabled by default because the current operator flow runs it on the main PC.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare


def profile_value(stable_low_latency, wired_360_default, precision_one_wrist, debug_raw):
    profile = LaunchConfiguration('teleop_feedback_profile')
    return PythonExpression([
        "'", profile, "' == 'stable_low_latency' and '", stable_low_latency, "' or ",
        "'", profile, "' == 'precision_one_wrist' and '", precision_one_wrist, "' or ",
        "'", profile, "' == 'debug_raw' and '", debug_raw, "' or '",
        wired_360_default, "'",
    ])


def wrist_high_value(default_value, high_value):
    high_profile = LaunchConfiguration('wrist_high_profile')
    return PythonExpression([
        "'", high_profile, "' == 'true' and '", high_value, "' or '", default_value, "'",
    ])


def generate_launch_description():
    leader_controller_config = LaunchConfiguration('leader_controller_config')
    leader_left_port = LaunchConfiguration('leader_left_port')
    leader_right_port = LaunchConfiguration('leader_right_port')
    init_position = LaunchConfiguration('init_position')
    launch_lidar = LaunchConfiguration('launch_lidar')
    start_feedback = LaunchConfiguration('start_feedback')
    start_leader = LaunchConfiguration('start_leader')

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ffw_bringup'),
                'launch',
                'ffw_sg2_follower_ai.launch.py',
            ])),
        launch_arguments={
            'launch_cameras': 'false',
            'launch_lidar': launch_lidar,
            'init_position': init_position,
            'start_rviz': 'false',
        }.items(),
    )

    feedback = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'launch',
                'teleop_wrist_depth.launch.py',
            ])),
        launch_arguments={
            'teleop_feedback_profile': LaunchConfiguration('teleop_feedback_profile'),
            'wrist_high_profile': LaunchConfiguration('wrist_high_profile'),
            'start_zed': LaunchConfiguration('start_zed'),
            'start_wrist_cameras': LaunchConfiguration('start_wrist_cameras'),
            'start_left_wrist': LaunchConfiguration('start_left_wrist'),
            'start_right_wrist': LaunchConfiguration('start_right_wrist'),
            'left_depth_profile': LaunchConfiguration('left_depth_profile'),
            'right_depth_profile': LaunchConfiguration('right_depth_profile'),
            'left_color_profile': LaunchConfiguration('left_color_profile'),
            'right_color_profile': LaunchConfiguration('right_color_profile'),
            'enable_left_color': LaunchConfiguration('enable_left_color'),
            'enable_right_color': LaunchConfiguration('enable_right_color'),
            'start_overlay': LaunchConfiguration('start_right_overlay'),
            'start_left_overlay': LaunchConfiguration('start_left_overlay'),
            'overlay_fps': LaunchConfiguration('overlay_fps'),
            'publish_raw_overlay': LaunchConfiguration('publish_raw_overlay'),
            'publish_base_compressed': LaunchConfiguration('publish_base_compressed'),
            'base_compressed_fps': LaunchConfiguration('base_compressed_fps'),
            'base_compressed_jpeg_quality': LaunchConfiguration('base_compressed_jpeg_quality'),
            'right_wrist_start_delay_s': LaunchConfiguration('right_wrist_start_delay_s'),
            'start_bandwidth_monitor': LaunchConfiguration('start_bandwidth_monitor'),
            'bandwidth_available_mbps': LaunchConfiguration('bandwidth_available_mbps'),
            'bandwidth_usb_available_mbps': LaunchConfiguration('bandwidth_usb_available_mbps'),
            'bandwidth_panel_topic': LaunchConfiguration('bandwidth_panel_topic'),
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'leader_controller_config',
            default_value='ffw_lg2_leader_ai_hardware_controller.yaml'),
        DeclareLaunchArgument('leader_left_port', default_value='/dev/left_leader'),
        DeclareLaunchArgument('leader_right_port', default_value='/dev/right_leader'),
        DeclareLaunchArgument('init_position', default_value='true'),
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('start_feedback', default_value='true'),
        DeclareLaunchArgument('start_leader', default_value='false'),
        DeclareLaunchArgument('start_zed', default_value='true'),
        DeclareLaunchArgument('start_wrist_cameras', default_value='true'),
        DeclareLaunchArgument('start_left_wrist', default_value='true'),
        DeclareLaunchArgument('start_right_wrist', default_value='true'),
        DeclareLaunchArgument('start_left_overlay', default_value='true'),
        DeclareLaunchArgument('start_right_overlay', default_value='true'),
        DeclareLaunchArgument('teleop_feedback_profile', default_value='wired_360_default'),
        DeclareLaunchArgument('wrist_high_profile', default_value='false'),
        DeclareLaunchArgument('left_depth_profile', default_value='480,270,15'),
        DeclareLaunchArgument('right_depth_profile', default_value='480,270,30'),
        DeclareLaunchArgument('left_color_profile', default_value='424,240,15'),
        DeclareLaunchArgument('right_color_profile', default_value='424,240,15'),
        DeclareLaunchArgument('enable_left_color', default_value='false'),
        DeclareLaunchArgument('enable_right_color', default_value='true'),
        DeclareLaunchArgument(
            'overlay_fps',
            default_value=profile_value('10.0', '30.0', '30.0', '30.0')),
        DeclareLaunchArgument(
            'publish_raw_overlay',
            default_value=profile_value('false', 'false', 'false', 'true')),
        DeclareLaunchArgument(
            'publish_base_compressed',
            default_value='false'),
        DeclareLaunchArgument('base_compressed_fps', default_value='5.0'),
        DeclareLaunchArgument('base_compressed_jpeg_quality', default_value='60'),
        DeclareLaunchArgument('right_wrist_start_delay_s', default_value='15.0'),
        DeclareLaunchArgument('start_bandwidth_monitor', default_value='true'),
        DeclareLaunchArgument('bandwidth_available_mbps', default_value='350.0'),
        DeclareLaunchArgument('bandwidth_usb_available_mbps', default_value='320.0'),
        DeclareLaunchArgument(
            'bandwidth_panel_topic', default_value='/teleop/bandwidth_monitor/compressed'),
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
