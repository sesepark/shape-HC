#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Sungho Woo, Woojin Wie, Wonho Yun

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value='ffw_lg2_leader.urdf.xacro',
            description='URDF/XACRO file for the robot model.',
        ),
        DeclareLaunchArgument(
            'leader_controller_config',
            default_value='ffw_lg2_leader_ai_hardware_controller.yaml',
            description='Controller YAML file for the LG2 Leader.',
        ),
        DeclareLaunchArgument(
            'leader_left_port',
            default_value='/dev/left_leader',
            description='Serial device for the left LG2 leader Dynamixel bus.',
        ),
        DeclareLaunchArgument(
            'leader_right_port',
            default_value='/dev/right_leader',
            description='Serial device for the right LG2 leader Dynamixel bus.',
        ),
        DeclareLaunchArgument(
            'publish_robot_description_topic',
            default_value='false',
            description='Start the legacy robot_description topic publisher.',
        ),
    ]

    description_file = LaunchConfiguration('description_file')
    leader_controller_config = LaunchConfiguration('leader_controller_config')
    leader_left_port = LaunchConfiguration('leader_left_port')
    leader_right_port = LaunchConfiguration('leader_right_port')
    publish_robot_description_topic = LaunchConfiguration('publish_robot_description_topic')
    leader_namespace = 'leader'
    leader_robot_description_topic = '/leader/robot_description'

    # Robot controllers config file path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ffw_bringup'),
            'config',
            'ffw_lg2_leader',
            leader_controller_config,
        ]
    )

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution(
                    [FindPackageShare('ffw_description'), 'urdf', 'ffw_lg2_leader', description_file]
                ),
                ' ',
                'leader_left_port:=', leader_left_port,
                ' ',
                'leader_right_port:=', leader_right_port,
            ]
        ),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    robot_description_topic_publisher_node = Node(
        package='ffw_bringup',
        executable='robot_description_topic_publisher',
        namespace=leader_namespace,
        output='both',
        parameters=[robot_description, {'publish_period_sec': 0.0}],
        remappings=[
            ('robot_description', leader_robot_description_topic),
        ],
        condition=IfCondition(publish_robot_description_topic),
    )

    # ros2_control Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=leader_namespace,
        parameters=[robot_description, robot_controllers],
        remappings=[
            ('~/robot_description', leader_robot_description_topic),
            ('robot_description', leader_robot_description_topic),
        ],
        output='both',
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=leader_namespace,
        arguments=[
            'joint_trajectory_command_broadcaster',
            'spring_actuator_controller_left',
            'spring_actuator_controller_right',
            'joystick_controller',
            'joint_state_broadcaster',
            '--controller-manager',
            '/leader/controller_manager',
        ],
        parameters=[robot_description],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=leader_namespace,
        output='both',
        parameters=[
            robot_description,
            {
                'frame_prefix': 'leader_',
                'use_robot_description_topic': False,
            },
        ],
        remappings=[
            ('robot_description', leader_robot_description_topic),
        ],
    )

    # Return combined LaunchDescription
    return LaunchDescription(
        declared_arguments + [
            LogInfo(msg=['LG2 Leader controller config: ', leader_controller_config]),
            LogInfo(msg=['LG2 Leader robot_description topic: ', leader_robot_description_topic]),
            LogInfo(msg=['LG2 Leader left port: ', leader_left_port]),
            LogInfo(msg=['LG2 Leader right port: ', leader_right_port]),
            robot_description_topic_publisher_node,
            robot_state_publisher_node,
            control_node,
            robot_controller_spawner,
        ]
    )
