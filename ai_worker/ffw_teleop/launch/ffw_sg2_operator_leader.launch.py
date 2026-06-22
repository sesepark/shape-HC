#!/usr/bin/env python3
"""Operator-side SG2 teleoperation launch.

Runs the RViz operator view and the LG2 leader controller on the main PC.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_file = LaunchConfiguration('description_file')
    leader_controller_config = LaunchConfiguration('leader_controller_config')
    leader_left_port = LaunchConfiguration('leader_left_port')
    leader_right_port = LaunchConfiguration('leader_right_port')
    start_rviz = LaunchConfiguration('start_rviz')
    start_leader = LaunchConfiguration('start_leader')
    start_mission_control = LaunchConfiguration('start_mission_control')
    start_operator_image_viewer = LaunchConfiguration('start_operator_image_viewer')
    start_operator_layout = LaunchConfiguration('start_operator_layout')
    operator_image_viewer_layout_store_path = LaunchConfiguration(
        'operator_image_viewer_layout_store_path')
    operator_image_viewer_canvas_width = LaunchConfiguration(
        'operator_image_viewer_canvas_width')
    operator_image_viewer_canvas_height = LaunchConfiguration(
        'operator_image_viewer_canvas_height')
    operator_image_viewer_auto_canvas_size = LaunchConfiguration(
        'operator_image_viewer_auto_canvas_size')
    operator_image_viewer_follow_window_size = LaunchConfiguration(
        'operator_image_viewer_follow_window_size')
    operator_image_viewer_show_toolbar = LaunchConfiguration(
        'operator_image_viewer_show_toolbar')
    rviz_config = LaunchConfiguration('rviz_config')
    mission_profiles_config = LaunchConfiguration('mission_profiles_config')
    operator_screen_layout_config = LaunchConfiguration('operator_screen_layout_config')
    operator_layout_action = LaunchConfiguration('operator_layout_action')
    operator_layout_store_path = LaunchConfiguration('operator_layout_store_path')
    leader_namespace = 'leader'
    leader_robot_description_topic = '/leader/robot_description'

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('ffw_bringup'),
        'config',
        'ffw_lg2_leader',
        leader_controller_config,
    ])

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('ffw_description'),
                'urdf',
                'ffw_lg2_leader',
                description_file,
            ]),
            ' ',
            'leader_left_port:=',
            leader_left_port,
            ' ',
            'leader_right_port:=',
            leader_right_port,
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    leader_robot_state_publisher = Node(
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
        condition=IfCondition(start_leader),
    )

    leader_control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=leader_namespace,
        parameters=[robot_description, robot_controllers],
        remappings=[
            ('~/robot_description', leader_robot_description_topic),
            ('robot_description', leader_robot_description_topic),
        ],
        output='both',
        condition=IfCondition(start_leader),
    )

    leader_spawner = Node(
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
        condition=IfCondition(start_leader),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='teleop_operator_rviz',
        arguments=['-d', rviz_config],
        output='screen',
        emulate_tty=True,
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1',
            'QT_OPENGL': 'software',
            'QT_X11_NO_MITSHM': '1',
        },
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

    operator_image_viewer = Node(
        package='ffw_teleop',
        executable='operator_image_viewer',
        name='operator_image_viewer',
        output='screen',
        parameters=[{
            'window_title': 'Teleop Image Viewer',
            'tile_width': 640,
            'tile_height': 360,
            'columns': 2,
            'canvas_width': operator_image_viewer_canvas_width,
            'canvas_height': operator_image_viewer_canvas_height,
            'auto_canvas_size': operator_image_viewer_auto_canvas_size,
            'follow_window_size': operator_image_viewer_follow_window_size,
            'layout_store_path': operator_image_viewer_layout_store_path,
            'show_toolbar': operator_image_viewer_show_toolbar,
            'window_x': 1520,
            'window_y': 40,
        }],
        condition=IfCondition(start_operator_image_viewer),
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
            'description_file',
            default_value='ffw_lg2_leader.urdf.xacro'),
        DeclareLaunchArgument(
            'leader_controller_config',
            default_value='ffw_lg2_leader_ai_hardware_controller.yaml'),
        DeclareLaunchArgument('leader_left_port', default_value='/dev/left_leader'),
        DeclareLaunchArgument('leader_right_port', default_value='/dev/right_leader'),
        DeclareLaunchArgument('start_leader', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_mission_control', default_value='true'),
        DeclareLaunchArgument('start_operator_image_viewer', default_value='true'),
        DeclareLaunchArgument(
            'operator_image_viewer_layout_store_path',
            default_value='~/.config/ffw_teleop/operator_image_viewer_layout.json'),
        DeclareLaunchArgument('operator_image_viewer_canvas_width', default_value='0'),
        DeclareLaunchArgument('operator_image_viewer_canvas_height', default_value='0'),
        DeclareLaunchArgument('operator_image_viewer_auto_canvas_size', default_value='true'),
        DeclareLaunchArgument('operator_image_viewer_follow_window_size', default_value='true'),
        DeclareLaunchArgument('operator_image_viewer_show_toolbar', default_value='true'),
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
        LogInfo(msg=['LG2 Leader controller config: ', leader_controller_config],
                condition=IfCondition(start_leader)),
        LogInfo(msg=['LG2 Leader robot_description topic: ', leader_robot_description_topic],
                condition=IfCondition(start_leader)),
        LogInfo(msg=['LG2 Leader left port: ', leader_left_port],
                condition=IfCondition(start_leader)),
        LogInfo(msg=['LG2 Leader right port: ', leader_right_port],
                condition=IfCondition(start_leader)),
        leader_robot_state_publisher,
        leader_control,
        leader_spawner,
        rviz,
        TimerAction(period=1.5, actions=[mission_control]),
        TimerAction(period=2.5, actions=[operator_image_viewer]),
        TimerAction(period=4.0, actions=[operator_layout]),
    ])
