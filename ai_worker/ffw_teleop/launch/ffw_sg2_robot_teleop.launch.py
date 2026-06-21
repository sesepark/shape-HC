#!/usr/bin/env python3
"""Robot-side SG2 teleoperation launch.

Runs follower control and robot-local camera feedback only. The LG2 leader
controller is expected to run on the operator PC.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
        }.items(),
    )

    return LaunchDescription([robot_teleop])
