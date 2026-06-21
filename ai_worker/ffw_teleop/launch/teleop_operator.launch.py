"""Operator-side RViz launch for SG2 teleoperation practice."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration('rviz_config')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='teleop_operator_rviz',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'rviz',
                'teleop_operator.rviz',
            ]),
        ),
        rviz,
    ])
