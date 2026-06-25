"""Teleoperation camera/depth feedback for ZED and selectable wrist cameras."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r', encoding='utf-8') as stream:
        return yaml.load(stream, Loader=yaml.SafeLoader) or {}


def wrist_serial_defaults():
    serials_path = os.path.join(
        get_package_share_directory('ffw_teleop'), 'config', 'wrist_realsense_serials.yaml')
    serials = yaml_to_dict(serials_path)
    return (
        serials.get('camera1_serial', ''),
        serials.get('camera2_serial', ''),
    )


def make_wrist_camera_launch(condition):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'launch',
                'camera_realsense_teleop.launch.py',
            ]),
        ),
        condition=condition,
        launch_arguments={
            'start_left_wrist': LaunchConfiguration('start_left_wrist'),
            'start_right_wrist': LaunchConfiguration('start_right_wrist'),
            'serial_no1': LaunchConfiguration('left_wrist_serial_no'),
            'serial_no2': LaunchConfiguration('right_wrist_serial_no'),
            'depth_module.depth_profile1': LaunchConfiguration('left_depth_profile'),
            'depth_module.depth_profile2': LaunchConfiguration('right_depth_profile'),
            'depth_module.color_profile1': LaunchConfiguration('left_color_profile'),
            'depth_module.color_profile2': LaunchConfiguration('right_color_profile'),
            'rgb_camera.color_profile1': LaunchConfiguration('left_color_profile'),
            'rgb_camera.color_profile2': LaunchConfiguration('right_color_profile'),
            'enable_depth1': 'true',
            'enable_depth2': LaunchConfiguration('enable_right_depth'),
            'enable_color1': LaunchConfiguration('enable_left_color'),
            'enable_color2': LaunchConfiguration('enable_right_color'),
            'align_depth.enable1': LaunchConfiguration('enable_left_align_depth'),
            'align_depth.enable2': LaunchConfiguration('enable_right_align_depth'),
            'pointcloud.enable1': 'false',
            'pointcloud.enable2': 'false',
            'colorizer.enable1': 'false',
            'colorizer.enable2': 'false',
            'left_wrist_start_delay_s': LaunchConfiguration('left_wrist_start_delay_s'),
            'right_wrist_start_delay_s': LaunchConfiguration('right_wrist_start_delay_s'),
        }.items(),
    )


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
    left_wrist_serial_no, right_wrist_serial_no = wrist_serial_defaults()
    start_zed = LaunchConfiguration('start_zed')
    start_zed_depth_assist = LaunchConfiguration('start_zed_depth_assist')
    start_overlay = LaunchConfiguration('start_overlay')
    start_left_overlay = LaunchConfiguration('start_left_overlay')
    start_alignment_monitor = LaunchConfiguration('start_alignment_monitor')
    start_bandwidth_monitor = LaunchConfiguration('start_bandwidth_monitor')

    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ffw_bringup'),
                'launch',
                'camera_zed.launch.py',
            ]),
        ),
        condition=IfCondition(start_zed),
        launch_arguments={
            'camera_model': LaunchConfiguration('zed_camera_model'),
            'camera_name': LaunchConfiguration('zed_camera_name'),
            'ros_params_override_path': LaunchConfiguration('zed_ros_params_override_path'),
        }.items(),
    )
    wrist_camera_launch_after_zed = TimerAction(
        period=10.0,
        condition=IfCondition(start_zed),
        actions=[
            make_wrist_camera_launch(IfCondition(LaunchConfiguration('start_wrist_cameras'))),
        ],
    )
    wrist_camera_launch_without_zed = make_wrist_camera_launch(
        IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('start_zed'),
                "' == 'false' and '",
                LaunchConfiguration('start_wrist_cameras'),
                "' == 'true'",
            ])
        )
    )

    zed_depth_assist_node = Node(
        package='ffw_teleop',
        executable='zed_depth_assist',
        name='zed_depth_assist',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'",
                start_zed,
                "' == 'true' and '",
                start_zed_depth_assist,
                "' == 'true'",
            ])
        ),
        parameters=[{
            'depth_topic': LaunchConfiguration('zed_depth_topic'),
            'base_image_topic': LaunchConfiguration('zed_base_image_topic'),
            'camera_info_topic': LaunchConfiguration('zed_camera_info_topic'),
            'assist_topic': LaunchConfiguration('zed_assist_topic'),
            'metrics_topic': LaunchConfiguration('zed_metrics_topic'),
            'stream_stats_topic': LaunchConfiguration('stream_stats_topic'),
            'camera_perf_topic': LaunchConfiguration('camera_perf_topic'),
            'stream_stats_name': 'zed',
            'operator_ok_topic': LaunchConfiguration('operator_ok_topic'),
            'ok_overlay_duration_s': LaunchConfiguration('ok_overlay_duration_s'),
            'assist_mode': LaunchConfiguration('zed_assist_mode'),
            'publish_fps': LaunchConfiguration('zed_assist_fps'),
            'jpeg_quality': LaunchConfiguration('zed_assist_jpeg_quality'),
            'min_depth_m': LaunchConfiguration('zed_min_depth_m'),
            'max_depth_m': LaunchConfiguration('zed_max_depth_m'),
            'base_image_timeout_s': LaunchConfiguration('zed_base_image_timeout_s'),
            'camera_optical_frame': LaunchConfiguration('zed_camera_optical_frame'),
            'use_latest_tf': LaunchConfiguration('zed_use_latest_tf'),
            'tf_lookup_timeout_s': LaunchConfiguration('zed_tf_lookup_timeout_s'),
            'left_hand_frame': LaunchConfiguration('zed_left_hand_frame'),
            'right_hand_frame': LaunchConfiguration('zed_right_hand_frame'),
            'left_arm_links': [
                'arm_l_link4',
                'arm_l_link5',
                'arm_l_link6',
                'arm_l_link7',
                'end_effector_l_link',
            ],
            'right_arm_links': [
                'arm_r_link4',
                'arm_r_link5',
                'arm_r_link6',
                'arm_r_link7',
                'end_effector_r_link',
            ],
            'enable_near_hand_objects': LaunchConfiguration('zed_enable_near_hand_objects'),
            'hand_roi_radius_px': LaunchConfiguration('zed_hand_roi_radius_px'),
            'robot_mask_radius_px': LaunchConfiguration('zed_robot_mask_radius_px'),
            'robot_mask_dilate_px': LaunchConfiguration('zed_robot_mask_dilate_px'),
            'near_hand_radius_m': LaunchConfiguration('zed_near_hand_radius_m'),
            'max_objects_per_hand': LaunchConfiguration('zed_max_objects_per_hand'),
            'component_min_area_px': LaunchConfiguration('zed_component_min_area_px'),
        }],
    )

    right_overlay_node = Node(
        package='ffw_teleop',
        executable='right_wrist_depth_overlay',
        name='right_wrist_depth_overlay',
        output='screen',
        condition=IfCondition(start_overlay),
        parameters=[{
            'depth_topic': LaunchConfiguration('depth_topic'),
            'base_image_topic': LaunchConfiguration('base_image_topic'),
            'overlay_topic': LaunchConfiguration('overlay_topic'),
            'compressed_topic': LaunchConfiguration('compressed_topic'),
            'assist_topic': LaunchConfiguration('assist_topic'),
            'color_compressed_topic': LaunchConfiguration('right_color_compressed_topic'),
            'base_compressed_topic': LaunchConfiguration('base_compressed_topic'),
            'center_distance_topic': LaunchConfiguration('center_distance_topic'),
            'metrics_topic': LaunchConfiguration('metrics_topic'),
            'stream_stats_topic': LaunchConfiguration('stream_stats_topic'),
            'camera_perf_topic': LaunchConfiguration('camera_perf_topic'),
            'stream_stats_name': 'wrist_right',
            'side': 'right',
            'feedback_visual_mode': LaunchConfiguration('feedback_visual_mode'),
            'subscribe_base_image': LaunchConfiguration('subscribe_base_image'),
            'publish_raw_overlay': LaunchConfiguration('publish_raw_overlay'),
            'publish_base_compressed': LaunchConfiguration('publish_base_compressed'),
            'base_compressed_fps': LaunchConfiguration('base_compressed_fps'),
            'base_compressed_jpeg_quality': LaunchConfiguration('base_compressed_jpeg_quality'),
            'publish_metrics': LaunchConfiguration('publish_metrics'),
            'publish_fps': LaunchConfiguration('overlay_fps'),
            'depth_scale': LaunchConfiguration('depth_scale'),
            'min_depth_m': LaunchConfiguration('min_depth_m'),
            'max_depth_m': LaunchConfiguration('max_depth_m'),
            'roi_size_px': LaunchConfiguration('roi_size_px'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            'colormap': LaunchConfiguration('colormap'),
            'depth_colormap': LaunchConfiguration('depth_colormap'),
            'base_alpha': LaunchConfiguration('base_alpha'),
            'depth_alpha': LaunchConfiguration('depth_alpha'),
            'base_image_timeout_s': LaunchConfiguration('base_image_timeout_s'),
            'show_depth_contours': LaunchConfiguration('show_depth_contours'),
            'contour_near_depth_m': LaunchConfiguration('contour_near_depth_m'),
            'contour_min_area_px': LaunchConfiguration('contour_min_area_px'),
            'invalid_depth_mode': LaunchConfiguration('invalid_depth_mode'),
            'assist_component_margin_m': LaunchConfiguration('assist_component_margin_m'),
            'assist_offset_threshold_px': LaunchConfiguration('assist_offset_threshold_px'),
            'assist_target_depth_m': LaunchConfiguration('assist_target_depth_m'),
            'assist_depth_tolerance_m': LaunchConfiguration('assist_depth_tolerance_m'),
            'view_preset': LaunchConfiguration('right_view_preset'),
            'view_rotate_deg': LaunchConfiguration('right_view_rotate_deg'),
            'view_flip_horizontal': LaunchConfiguration('right_view_flip_horizontal'),
            'view_flip_vertical': LaunchConfiguration('right_view_flip_vertical'),
            'gripper_target_offset_x_px': LaunchConfiguration('right_gripper_target_offset_x_px'),
            'gripper_target_offset_y_px': LaunchConfiguration('right_gripper_target_offset_y_px'),
            'band_red_max_m': LaunchConfiguration('band_red_max_m'),
            'band_green_min_m': LaunchConfiguration('band_green_min_m'),
            'band_green_max_m': LaunchConfiguration('band_green_max_m'),
            'band_orange_min_m': LaunchConfiguration('band_orange_min_m'),
            'band_orange_max_m': LaunchConfiguration('band_orange_max_m'),
            'band_alpha': LaunchConfiguration('band_alpha'),
            'band_min_area_px': LaunchConfiguration('band_min_area_px'),
        }],
    )

    left_overlay_node = Node(
        package='ffw_teleop',
        executable='right_wrist_depth_overlay',
        name='left_wrist_depth_overlay',
        output='screen',
        condition=IfCondition(start_left_overlay),
        parameters=[{
            'depth_topic': LaunchConfiguration('left_depth_topic'),
            'base_image_topic': LaunchConfiguration('left_base_image_topic'),
            'overlay_topic': LaunchConfiguration('left_overlay_topic'),
            'compressed_topic': LaunchConfiguration('left_compressed_topic'),
            'assist_topic': LaunchConfiguration('left_assist_topic'),
            'color_compressed_topic': LaunchConfiguration('left_color_compressed_topic'),
            'base_compressed_topic': LaunchConfiguration('left_base_compressed_topic'),
            'center_distance_topic': LaunchConfiguration('left_center_distance_topic'),
            'metrics_topic': LaunchConfiguration('left_metrics_topic'),
            'stream_stats_topic': LaunchConfiguration('stream_stats_topic'),
            'camera_perf_topic': LaunchConfiguration('camera_perf_topic'),
            'stream_stats_name': 'wrist_left',
            'side': 'left',
            'feedback_visual_mode': LaunchConfiguration('feedback_visual_mode'),
            'subscribe_base_image': LaunchConfiguration('subscribe_base_image'),
            'publish_raw_overlay': LaunchConfiguration('publish_raw_overlay'),
            'publish_base_compressed': LaunchConfiguration('publish_base_compressed'),
            'base_compressed_fps': LaunchConfiguration('base_compressed_fps'),
            'base_compressed_jpeg_quality': LaunchConfiguration('base_compressed_jpeg_quality'),
            'publish_metrics': LaunchConfiguration('publish_metrics'),
            'publish_fps': LaunchConfiguration('overlay_fps'),
            'depth_scale': LaunchConfiguration('depth_scale'),
            'min_depth_m': LaunchConfiguration('min_depth_m'),
            'max_depth_m': LaunchConfiguration('max_depth_m'),
            'roi_size_px': LaunchConfiguration('roi_size_px'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            'colormap': LaunchConfiguration('colormap'),
            'depth_colormap': LaunchConfiguration('depth_colormap'),
            'base_alpha': LaunchConfiguration('base_alpha'),
            'depth_alpha': LaunchConfiguration('depth_alpha'),
            'base_image_timeout_s': LaunchConfiguration('base_image_timeout_s'),
            'show_depth_contours': LaunchConfiguration('show_depth_contours'),
            'contour_near_depth_m': LaunchConfiguration('contour_near_depth_m'),
            'contour_min_area_px': LaunchConfiguration('contour_min_area_px'),
            'invalid_depth_mode': LaunchConfiguration('invalid_depth_mode'),
            'assist_component_margin_m': LaunchConfiguration('assist_component_margin_m'),
            'assist_offset_threshold_px': LaunchConfiguration('assist_offset_threshold_px'),
            'assist_target_depth_m': LaunchConfiguration('assist_target_depth_m'),
            'assist_depth_tolerance_m': LaunchConfiguration('assist_depth_tolerance_m'),
            'view_preset': LaunchConfiguration('left_view_preset'),
            'view_rotate_deg': LaunchConfiguration('left_view_rotate_deg'),
            'view_flip_horizontal': LaunchConfiguration('left_view_flip_horizontal'),
            'view_flip_vertical': LaunchConfiguration('left_view_flip_vertical'),
            'gripper_target_offset_x_px': LaunchConfiguration('left_gripper_target_offset_x_px'),
            'gripper_target_offset_y_px': LaunchConfiguration('left_gripper_target_offset_y_px'),
            'band_red_max_m': LaunchConfiguration('band_red_max_m'),
            'band_green_min_m': LaunchConfiguration('band_green_min_m'),
            'band_green_max_m': LaunchConfiguration('band_green_max_m'),
            'band_orange_min_m': LaunchConfiguration('band_orange_min_m'),
            'band_orange_max_m': LaunchConfiguration('band_orange_max_m'),
            'band_alpha': LaunchConfiguration('band_alpha'),
            'band_min_area_px': LaunchConfiguration('band_min_area_px'),
        }],
    )

    alignment_node = Node(
        package='ffw_teleop',
        executable='teleop_alignment_status',
        name='teleop_alignment_status',
        output='screen',
        condition=IfCondition(start_alignment_monitor),
        parameters=[{
            'right_goal_topic': LaunchConfiguration('right_goal_topic'),
            'left_goal_topic': LaunchConfiguration('left_goal_topic'),
            'right_current_topic': LaunchConfiguration('right_current_topic'),
            'left_current_topic': LaunchConfiguration('left_current_topic'),
            'status_topic': LaunchConfiguration('alignment_status_topic'),
            'ok_topic': LaunchConfiguration('alignment_ok_topic'),
            'publish_hz': LaunchConfiguration('alignment_publish_hz'),
            'pos_threshold_m': LaunchConfiguration('pos_threshold_m'),
            'ori_threshold_deg': LaunchConfiguration('ori_threshold_deg'),
            'stale_timeout_s': LaunchConfiguration('stale_timeout_s'),
            'record_practice_events': LaunchConfiguration('record_practice_events'),
            'practice_event_log_path': LaunchConfiguration('practice_event_log_path'),
            'practice_event_input_topic': LaunchConfiguration('practice_event_input_topic'),
            'practice_event_output_topic': LaunchConfiguration('practice_event_output_topic'),
            'tact_trigger_topic': LaunchConfiguration('tact_trigger_topic'),
            'joystick_mode_topic': LaunchConfiguration('joystick_mode_topic'),
            'head_target_topic': LaunchConfiguration('head_target_topic'),
            'joint_state_topic': LaunchConfiguration('joint_state_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'right_center_distance_topic': LaunchConfiguration('center_distance_topic'),
            'left_center_distance_topic': LaunchConfiguration('left_center_distance_topic'),
            'right_depth_metrics_topic': LaunchConfiguration('metrics_topic'),
            'left_depth_metrics_topic': LaunchConfiguration('left_metrics_topic'),
            'status_panel_topic': LaunchConfiguration('status_panel_topic'),
            'status_panel_jpeg_quality': LaunchConfiguration('status_panel_jpeg_quality'),
            'table_reference_enabled': LaunchConfiguration('table_reference_enabled'),
            'table_x_m': LaunchConfiguration('table_x_m'),
            'table_y_m': LaunchConfiguration('table_y_m'),
            'table_yaw_deg': LaunchConfiguration('table_yaw_deg'),
        }],
    )

    bandwidth_monitor_node = Node(
        package='ffw_teleop',
        executable='teleop_bandwidth_monitor',
        name='teleop_bandwidth_monitor',
        output='screen',
        condition=IfCondition(start_bandwidth_monitor),
        parameters=[{
            'stream_stats_topic': LaunchConfiguration('stream_stats_topic'),
            'camera_perf_topic': LaunchConfiguration('camera_perf_topic'),
            'monitor_topic': LaunchConfiguration('bandwidth_monitor_topic'),
            'panel_topic': LaunchConfiguration('bandwidth_panel_topic'),
            'available_mbps': LaunchConfiguration('bandwidth_available_mbps'),
            'publish_hz': LaunchConfiguration('bandwidth_monitor_publish_hz'),
            'usb_available_mbps': LaunchConfiguration('bandwidth_usb_available_mbps'),
            'usb_wrist_left_depth_profile': LaunchConfiguration('left_depth_profile'),
            'usb_wrist_right_depth_profile': LaunchConfiguration('right_depth_profile'),
            'usb_wrist_left_color_profile': LaunchConfiguration('left_color_profile'),
            'usb_wrist_right_color_profile': LaunchConfiguration('right_color_profile'),
            'usb_wrist_right_depth_enabled': LaunchConfiguration('enable_right_depth'),
            'usb_wrist_left_color_enabled': LaunchConfiguration('enable_left_color'),
            'usb_wrist_right_color_enabled': LaunchConfiguration('enable_right_color'),
            'wrist_right_color_compressed_topic': LaunchConfiguration(
                'right_color_compressed_topic'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('teleop_feedback_profile', default_value='wired_360_default'),
        DeclareLaunchArgument('wrist_high_profile', default_value='false'),
        DeclareLaunchArgument('start_zed', default_value='true'),
        DeclareLaunchArgument('start_wrist_cameras', default_value='true'),
        DeclareLaunchArgument('start_left_wrist', default_value='true'),
        DeclareLaunchArgument('start_right_wrist', default_value='true'),
        DeclareLaunchArgument('start_overlay', default_value='false'),
        DeclareLaunchArgument('start_left_overlay', default_value='true'),
        DeclareLaunchArgument('start_alignment_monitor', default_value='true'),
        DeclareLaunchArgument('start_bandwidth_monitor', default_value='true'),
        DeclareLaunchArgument('start_zed_depth_assist', default_value='true'),
        DeclareLaunchArgument('zed_camera_model', default_value='zedm'),
        DeclareLaunchArgument('zed_camera_name', default_value='zed'),
        DeclareLaunchArgument(
            'zed_ros_params_override_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('ffw_teleop'),
                'config',
                'zed_teleop_depth.yaml',
            ])),
        DeclareLaunchArgument(
            'zed_depth_topic', default_value='/zed/zed_node/depth/depth_registered'),
        DeclareLaunchArgument(
            'zed_base_image_topic', default_value='/zed/zed_node/left/image_rect_color'),
        DeclareLaunchArgument(
            'zed_camera_info_topic', default_value='/zed/zed_node/left/camera_info'),
        DeclareLaunchArgument(
            'zed_assist_topic', default_value='/teleop/zed/depth_assist/compressed'),
        DeclareLaunchArgument(
            'zed_metrics_topic', default_value='/teleop/zed/depth_metrics'),
        DeclareLaunchArgument(
            'stream_stats_topic', default_value='/teleop/stream_stats'),
        DeclareLaunchArgument(
            'camera_perf_topic', default_value='/teleop/camera_perf'),
        DeclareLaunchArgument('operator_ok_topic', default_value='/teleop/operator_ok'),
        DeclareLaunchArgument('ok_overlay_duration_s', default_value='3.0'),
        DeclareLaunchArgument('zed_assist_mode', default_value='tf_header'),
        DeclareLaunchArgument('zed_assist_fps', default_value='30.0'),
        DeclareLaunchArgument('zed_assist_jpeg_quality', default_value='88'),
        DeclareLaunchArgument('zed_min_depth_m', default_value='0.15'),
        DeclareLaunchArgument('zed_max_depth_m', default_value='4.0'),
        DeclareLaunchArgument('zed_base_image_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('zed_camera_optical_frame', default_value=''),
        DeclareLaunchArgument('zed_use_latest_tf', default_value='true'),
        DeclareLaunchArgument('zed_tf_lookup_timeout_s', default_value='0.005'),
        DeclareLaunchArgument('zed_left_hand_frame', default_value='end_effector_l_link'),
        DeclareLaunchArgument('zed_right_hand_frame', default_value='end_effector_r_link'),
        DeclareLaunchArgument('zed_enable_near_hand_objects', default_value='false'),
        DeclareLaunchArgument('zed_hand_roi_radius_px', default_value='110'),
        DeclareLaunchArgument('zed_robot_mask_radius_px', default_value='22'),
        DeclareLaunchArgument('zed_robot_mask_dilate_px', default_value='18'),
        DeclareLaunchArgument('zed_near_hand_radius_m', default_value='0.35'),
        DeclareLaunchArgument('zed_max_objects_per_hand', default_value='3'),
        DeclareLaunchArgument('zed_component_min_area_px', default_value='80.0'),
        DeclareLaunchArgument(
            'left_depth_profile',
            default_value=wrist_high_value('480,270,5', '640,480,30')),
        DeclareLaunchArgument(
            'right_depth_profile',
            default_value=wrist_high_value('480,270,30', '640,480,30')),
        DeclareLaunchArgument(
            'left_color_profile',
            default_value=wrist_high_value('424,240,15', '640,480,30')),
        DeclareLaunchArgument(
            'right_color_profile',
            default_value=wrist_high_value('424,240,15', '640,480,30')),
        DeclareLaunchArgument('left_wrist_serial_no', default_value=left_wrist_serial_no),
        DeclareLaunchArgument('right_wrist_serial_no', default_value=right_wrist_serial_no),
        DeclareLaunchArgument(
            'enable_left_color',
            default_value=profile_value('false', 'false', 'false', 'true')),
        DeclareLaunchArgument(
            'enable_right_color',
            default_value=profile_value('true', 'true', 'true', 'true')),
        DeclareLaunchArgument('enable_right_depth', default_value='false'),
        DeclareLaunchArgument('enable_left_align_depth', default_value='false'),
        DeclareLaunchArgument('enable_right_align_depth', default_value='false'),
        DeclareLaunchArgument('left_wrist_start_delay_s', default_value='15.0'),
        DeclareLaunchArgument(
            'right_wrist_start_delay_s',
            default_value='0.0',
            description='Deprecated; right wrist launches first and is no longer delayed.'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera_right/camera_right/depth/image_rect_raw'),
        DeclareLaunchArgument(
            'base_image_topic',
            default_value='',
            description='Deprecated and ignored; raw wrist color subscriptions stay disabled.'),
        DeclareLaunchArgument(
            'overlay_topic', default_value='/teleop/wrist_right/depth_overlay'),
        DeclareLaunchArgument(
            'compressed_topic', default_value='/teleop/wrist_right/depth_overlay/compressed'),
        DeclareLaunchArgument(
            'assist_topic', default_value='/teleop/wrist_right/depth_assist/compressed'),
        DeclareLaunchArgument(
            'right_color_compressed_topic',
            default_value='/camera_right/camera_right/color/image_rect_raw/compressed'),
        DeclareLaunchArgument(
            'base_compressed_topic',
            default_value='/teleop/wrist_right/color/compressed',
            description='Deprecated and ignored; use right_color_compressed_topic.'),
        DeclareLaunchArgument(
            'center_distance_topic', default_value='/teleop/wrist_right/center_distance_m'),
        DeclareLaunchArgument(
            'metrics_topic', default_value='/teleop/wrist_right/depth_metrics'),
        DeclareLaunchArgument(
            'left_depth_topic',
            default_value='/camera_left/camera_left/depth/image_rect_raw'),
        DeclareLaunchArgument(
            'left_base_image_topic',
            default_value='',
            description='Deprecated and ignored; raw wrist color subscriptions stay disabled.'),
        DeclareLaunchArgument(
            'left_overlay_topic', default_value='/teleop/wrist_left/depth_overlay'),
        DeclareLaunchArgument(
            'left_compressed_topic', default_value='/teleop/wrist_left/depth_overlay/compressed'),
        DeclareLaunchArgument(
            'left_assist_topic', default_value='/teleop/wrist_left/depth_assist/compressed'),
        DeclareLaunchArgument(
            'left_color_compressed_topic',
            default_value='/camera_left/camera_left/color/image_rect_raw/compressed'),
        DeclareLaunchArgument(
            'left_base_compressed_topic',
            default_value='/teleop/wrist_left/color/compressed',
            description='Deprecated and ignored; use left_color_compressed_topic.'),
        DeclareLaunchArgument(
            'left_center_distance_topic', default_value='/teleop/wrist_left/center_distance_m'),
        DeclareLaunchArgument(
            'left_metrics_topic', default_value='/teleop/wrist_left/depth_metrics'),
        DeclareLaunchArgument(
            'feedback_visual_mode',
            default_value=profile_value('assist', 'assist', 'assist', 'overlay')),
        DeclareLaunchArgument(
            'publish_raw_overlay',
            default_value=profile_value('false', 'false', 'false', 'true')),
        DeclareLaunchArgument(
            'subscribe_base_image',
            default_value='false',
            description='Deprecated and ignored; raw wrist color subscriptions stay disabled.'),
        DeclareLaunchArgument(
            'publish_base_compressed',
            default_value='false',
            description='Deprecated and ignored; use RealSense compressed color transport.'),
        DeclareLaunchArgument('base_compressed_fps', default_value='5.0'),
        DeclareLaunchArgument('base_compressed_jpeg_quality', default_value='60'),
        DeclareLaunchArgument('publish_metrics', default_value='true'),
        DeclareLaunchArgument(
            'overlay_fps',
            default_value=profile_value('10.0', '35.0', '35.0', '35.0')),
        DeclareLaunchArgument('depth_scale', default_value='0.001'),
        DeclareLaunchArgument('min_depth_m', default_value='0.03'),
        DeclareLaunchArgument('max_depth_m', default_value='0.70'),
        DeclareLaunchArgument('roi_size_px', default_value='32'),
        DeclareLaunchArgument(
            'jpeg_quality',
            default_value=profile_value('70', '88', '88', '88')),
        DeclareLaunchArgument('colormap', default_value='VIRIDIS'),
        DeclareLaunchArgument('depth_colormap', default_value=''),
        DeclareLaunchArgument('base_alpha', default_value='0.70'),
        DeclareLaunchArgument('depth_alpha', default_value='0.30'),
        DeclareLaunchArgument('base_image_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('show_depth_contours', default_value='true'),
        DeclareLaunchArgument('contour_near_depth_m', default_value='0.45'),
        DeclareLaunchArgument('contour_min_area_px', default_value='30.0'),
        DeclareLaunchArgument('invalid_depth_mode', default_value='base_only'),
        DeclareLaunchArgument('assist_component_margin_m', default_value='0.08'),
        DeclareLaunchArgument('assist_offset_threshold_px', default_value='24'),
        DeclareLaunchArgument('assist_target_depth_m', default_value='0.30'),
        DeclareLaunchArgument('assist_depth_tolerance_m', default_value='0.04'),
        DeclareLaunchArgument('left_view_preset', default_value='driver_90'),
        DeclareLaunchArgument('right_view_preset', default_value='driver_90'),
        DeclareLaunchArgument('left_view_rotate_deg', default_value='90.0'),
        DeclareLaunchArgument('right_view_rotate_deg', default_value='90.0'),
        DeclareLaunchArgument('left_view_flip_horizontal', default_value='false'),
        DeclareLaunchArgument('right_view_flip_horizontal', default_value='false'),
        DeclareLaunchArgument('left_view_flip_vertical', default_value='false'),
        DeclareLaunchArgument('right_view_flip_vertical', default_value='false'),
        DeclareLaunchArgument('left_gripper_target_offset_x_px', default_value='0'),
        DeclareLaunchArgument('left_gripper_target_offset_y_px', default_value='96'),
        DeclareLaunchArgument('right_gripper_target_offset_x_px', default_value='0'),
        DeclareLaunchArgument('right_gripper_target_offset_y_px', default_value='96'),
        DeclareLaunchArgument('band_red_max_m', default_value='0.06'),
        DeclareLaunchArgument('band_green_min_m', default_value='0.06'),
        DeclareLaunchArgument('band_green_max_m', default_value='0.13'),
        DeclareLaunchArgument('band_orange_min_m', default_value='0.13'),
        DeclareLaunchArgument('band_orange_max_m', default_value='0.20'),
        DeclareLaunchArgument('band_alpha', default_value='0.45'),
        DeclareLaunchArgument('band_min_area_px', default_value='20.0'),
        DeclareLaunchArgument('right_goal_topic', default_value='/r_goal_pose'),
        DeclareLaunchArgument('left_goal_topic', default_value='/l_goal_pose'),
        DeclareLaunchArgument('right_current_topic', default_value='/r_gripper_pose'),
        DeclareLaunchArgument('left_current_topic', default_value='/l_gripper_pose'),
        DeclareLaunchArgument(
            'alignment_status_topic', default_value='/teleop/alignment_status'),
        DeclareLaunchArgument('alignment_ok_topic', default_value='/teleop/alignment_ok'),
        DeclareLaunchArgument('alignment_publish_hz', default_value='10.0'),
        DeclareLaunchArgument('pos_threshold_m', default_value='0.30'),
        DeclareLaunchArgument('ori_threshold_deg', default_value='120.0'),
        DeclareLaunchArgument('stale_timeout_s', default_value='1.0'),
        DeclareLaunchArgument('record_practice_events', default_value='true'),
        DeclareLaunchArgument(
            'practice_event_log_path', default_value='~/teleop_practice_events.jsonl'),
        DeclareLaunchArgument(
            'practice_event_input_topic', default_value='/teleop/practice_event/mark'),
        DeclareLaunchArgument(
            'practice_event_output_topic', default_value='/teleop/practice_event'),
        DeclareLaunchArgument(
            'tact_trigger_topic', default_value='/leader/joystick_controller/tact_trigger'),
        DeclareLaunchArgument(
            'joystick_mode_topic',
            default_value='/leader/joystick_controller_right/joystick_mode'),
        DeclareLaunchArgument(
            'head_target_topic',
            default_value='/leader/joystick_controller_left/joint_trajectory'),
        DeclareLaunchArgument('joint_state_topic', default_value='/joint_states'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'status_panel_topic', default_value='/teleop/operator_status/compressed'),
        DeclareLaunchArgument('status_panel_jpeg_quality', default_value='95'),
        DeclareLaunchArgument(
            'bandwidth_monitor_topic', default_value='/teleop/bandwidth_monitor'),
        DeclareLaunchArgument(
            'bandwidth_panel_topic', default_value='/teleop/bandwidth_monitor/compressed'),
        DeclareLaunchArgument('bandwidth_available_mbps', default_value='350.0'),
        DeclareLaunchArgument('bandwidth_usb_available_mbps', default_value='320.0'),
        DeclareLaunchArgument('bandwidth_monitor_publish_hz', default_value='2.0'),
        DeclareLaunchArgument('table_reference_enabled', default_value='false'),
        DeclareLaunchArgument('table_x_m', default_value='0.0'),
        DeclareLaunchArgument('table_y_m', default_value='0.0'),
        DeclareLaunchArgument('table_yaw_deg', default_value='0.0'),
        zed_camera_launch,
        zed_depth_assist_node,
        wrist_camera_launch_after_zed,
        wrist_camera_launch_without_zed,
        right_overlay_node,
        left_overlay_node,
        alignment_node,
        bandwidth_monitor_node,
    ])
