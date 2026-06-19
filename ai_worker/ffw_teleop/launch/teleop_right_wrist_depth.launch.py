"""Teleoperation camera/depth feedback for ZED and selectable wrist cameras."""

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


def make_wrist_camera_launch(condition):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ffw_bringup'),
                'launch',
                'camera_realsense_teleop.launch.py',
            ]),
        ),
        condition=condition,
        launch_arguments={
            'start_left_wrist': LaunchConfiguration('start_left_wrist'),
            'start_right_wrist': LaunchConfiguration('start_right_wrist'),
            'depth_module.depth_profile1': LaunchConfiguration('left_depth_profile'),
            'depth_module.depth_profile2': LaunchConfiguration('right_depth_profile'),
            'depth_module.color_profile1': LaunchConfiguration('left_color_profile'),
            'depth_module.color_profile2': LaunchConfiguration('right_color_profile'),
            'enable_depth1': 'true',
            'enable_depth2': 'true',
            'enable_color1': LaunchConfiguration('enable_left_color'),
            'enable_color2': LaunchConfiguration('enable_right_color'),
            'align_depth.enable1': 'false',
            'align_depth.enable2': 'false',
            'pointcloud.enable1': 'false',
            'pointcloud.enable2': 'false',
            'colorizer.enable1': 'false',
            'colorizer.enable2': 'false',
        }.items(),
    )


def generate_launch_description():
    start_zed = LaunchConfiguration('start_zed')
    start_overlay = LaunchConfiguration('start_overlay')
    start_left_overlay = LaunchConfiguration('start_left_overlay')
    start_alignment_monitor = LaunchConfiguration('start_alignment_monitor')

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
            'center_distance_topic': LaunchConfiguration('center_distance_topic'),
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
            'center_distance_topic': LaunchConfiguration('left_center_distance_topic'),
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
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_zed', default_value='true'),
        DeclareLaunchArgument('start_wrist_cameras', default_value='true'),
        DeclareLaunchArgument('start_left_wrist', default_value='false'),
        DeclareLaunchArgument('start_right_wrist', default_value='true'),
        DeclareLaunchArgument('start_overlay', default_value='true'),
        DeclareLaunchArgument('start_left_overlay', default_value='false'),
        DeclareLaunchArgument('start_alignment_monitor', default_value='true'),
        DeclareLaunchArgument('zed_camera_model', default_value='zedm'),
        DeclareLaunchArgument('zed_camera_name', default_value='zed'),
        DeclareLaunchArgument('zed_ros_params_override_path', default_value=''),
        DeclareLaunchArgument('left_depth_profile', default_value='480,270,15'),
        DeclareLaunchArgument('right_depth_profile', default_value='480,270,15'),
        DeclareLaunchArgument('left_color_profile', default_value='424,240,15'),
        DeclareLaunchArgument('right_color_profile', default_value='424,240,15'),
        DeclareLaunchArgument('enable_left_color', default_value='true'),
        DeclareLaunchArgument('enable_right_color', default_value='true'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera_right/camera_right/depth/image_rect_raw'),
        DeclareLaunchArgument(
            'base_image_topic', default_value='/camera_right/camera_right/color/image_raw'),
        DeclareLaunchArgument(
            'overlay_topic', default_value='/teleop/wrist_right/depth_overlay'),
        DeclareLaunchArgument(
            'compressed_topic', default_value='/teleop/wrist_right/depth_overlay/compressed'),
        DeclareLaunchArgument(
            'center_distance_topic', default_value='/teleop/wrist_right/center_distance_m'),
        DeclareLaunchArgument(
            'left_depth_topic', default_value='/camera_left/camera_left/depth/image_rect_raw'),
        DeclareLaunchArgument(
            'left_base_image_topic', default_value='/camera_left/camera_left/color/image_raw'),
        DeclareLaunchArgument(
            'left_overlay_topic', default_value='/teleop/wrist_left/depth_overlay'),
        DeclareLaunchArgument(
            'left_compressed_topic', default_value='/teleop/wrist_left/depth_overlay/compressed'),
        DeclareLaunchArgument(
            'left_center_distance_topic', default_value='/teleop/wrist_left/center_distance_m'),
        DeclareLaunchArgument('overlay_fps', default_value='10.0'),
        DeclareLaunchArgument('depth_scale', default_value='0.001'),
        DeclareLaunchArgument('min_depth_m', default_value='0.10'),
        DeclareLaunchArgument('max_depth_m', default_value='1.50'),
        DeclareLaunchArgument('roi_size_px', default_value='32'),
        DeclareLaunchArgument('jpeg_quality', default_value='70'),
        DeclareLaunchArgument('colormap', default_value='VIRIDIS'),
        DeclareLaunchArgument('depth_colormap', default_value=''),
        DeclareLaunchArgument('base_alpha', default_value='0.70'),
        DeclareLaunchArgument('depth_alpha', default_value='0.30'),
        DeclareLaunchArgument('base_image_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('show_depth_contours', default_value='true'),
        DeclareLaunchArgument('contour_near_depth_m', default_value='0.55'),
        DeclareLaunchArgument('contour_min_area_px', default_value='30.0'),
        DeclareLaunchArgument('invalid_depth_mode', default_value='base_only'),
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
        zed_camera_launch,
        wrist_camera_launch_after_zed,
        wrist_camera_launch_without_zed,
        right_overlay_node,
        left_overlay_node,
        alignment_node,
    ])
