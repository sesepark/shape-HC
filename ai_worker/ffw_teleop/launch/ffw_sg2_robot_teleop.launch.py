#!/usr/bin/env python3
"""Robot-side SG2 teleoperation launch.

Runs follower control and robot-local camera feedback only. The LG2 leader
controller is expected to run on the operator PC.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_bandwidth_monitor = LaunchConfiguration('start_bandwidth_monitor')
    bandwidth_available_mbps = LaunchConfiguration('bandwidth_available_mbps')
    bandwidth_usb_available_mbps = LaunchConfiguration('bandwidth_usb_available_mbps')
    bandwidth_panel_topic = LaunchConfiguration('bandwidth_panel_topic')
    left_wrist_start_delay_s = LaunchConfiguration('left_wrist_start_delay_s')
    right_wrist_start_delay_s = LaunchConfiguration('right_wrist_start_delay_s')
    wrist_high_profile = LaunchConfiguration('wrist_high_profile')
    left_depth_profile = LaunchConfiguration('left_depth_profile')
    right_depth_profile = LaunchConfiguration('right_depth_profile')
    left_color_profile = LaunchConfiguration('left_color_profile')
    right_color_profile = LaunchConfiguration('right_color_profile')
    enable_left_color = LaunchConfiguration('enable_left_color')
    enable_right_color = LaunchConfiguration('enable_right_color')
    base_compressed_fps = LaunchConfiguration('base_compressed_fps')
    base_compressed_jpeg_quality = LaunchConfiguration('base_compressed_jpeg_quality')

    robot_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'launch',
                'ffw_sg2_teleop_practice.launch.py',
            ])),
        launch_arguments={
            'start_leader': 'false',
            'start_feedback': 'true',
            'start_zed': 'true',
            'start_wrist_cameras': 'true',
            'teleop_feedback_profile': 'wired_360_default',
            'wrist_high_profile': wrist_high_profile,
            'left_depth_profile': left_depth_profile,
            'right_depth_profile': right_depth_profile,
            'left_color_profile': left_color_profile,
            'right_color_profile': right_color_profile,
            'enable_left_color': enable_left_color,
            'enable_right_color': enable_right_color,
            'start_bandwidth_monitor': start_bandwidth_monitor,
            'bandwidth_available_mbps': bandwidth_available_mbps,
            'bandwidth_usb_available_mbps': bandwidth_usb_available_mbps,
            'bandwidth_panel_topic': bandwidth_panel_topic,
            'left_wrist_start_delay_s': left_wrist_start_delay_s,
            'right_wrist_start_delay_s': right_wrist_start_delay_s,
            'base_compressed_fps': base_compressed_fps,
            'base_compressed_jpeg_quality': base_compressed_jpeg_quality,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_bandwidth_monitor', default_value='true'),
        DeclareLaunchArgument('bandwidth_available_mbps', default_value='350.0'),
        DeclareLaunchArgument('bandwidth_usb_available_mbps', default_value='320.0'),
        DeclareLaunchArgument(
            'bandwidth_panel_topic', default_value='/teleop/bandwidth_monitor/compressed'),
        DeclareLaunchArgument('left_wrist_start_delay_s', default_value='15.0'),
        DeclareLaunchArgument(
            'right_wrist_start_delay_s',
            default_value='0.0',
            description='Deprecated; right wrist launches first and is no longer delayed.'),
        DeclareLaunchArgument('wrist_high_profile', default_value='false'),
        DeclareLaunchArgument('left_depth_profile', default_value='480,270,5'),
        DeclareLaunchArgument('right_depth_profile', default_value='480,270,30'),
        DeclareLaunchArgument('left_color_profile', default_value='424,240,15'),
        DeclareLaunchArgument('right_color_profile', default_value='424,240,15'),
        DeclareLaunchArgument('enable_left_color', default_value='false'),
        DeclareLaunchArgument('enable_right_color', default_value='true'),
        DeclareLaunchArgument('base_compressed_fps', default_value='5.0'),
        DeclareLaunchArgument('base_compressed_jpeg_quality', default_value='60'),
        robot_teleop,
    ])
