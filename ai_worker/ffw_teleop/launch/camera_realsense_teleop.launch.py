# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Launch selectable wrist RealSense cameras for teleoperation."""
import copy
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import yaml

realsense2_camera_launch_dir = os.path.join(
    get_package_share_directory('realsense2_camera'), 'launch')
sys.path.append(realsense2_camera_launch_dir)
import rs_launch  # noqa: E402, I100


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


serials_path = os.path.join(
    get_package_share_directory('ffw_bringup'), 'config', 'common', 'rs_serial.yaml')
serials = yaml_to_dict(serials_path)
serial1 = serials.get('camera1_serial')
serial2 = serials.get('camera2_serial')

local_parameters = [
    {'name': 'start_left_wrist', 'default': 'false',
     'description': 'launch the left wrist RealSense camera'},
    {'name': 'start_right_wrist', 'default': 'true',
     'description': 'launch the right wrist RealSense camera'},
    {'name': 'camera_name1', 'default': 'camera_left',
     'description': 'left wrist camera unique name'},
    {'name': 'camera_name2', 'default': 'camera_right',
     'description': 'right wrist camera unique name'},
    {'name': 'camera_namespace1', 'default': 'camera_left',
     'description': 'left wrist camera namespace'},
    {'name': 'camera_namespace2', 'default': 'camera_right',
     'description': 'right wrist camera namespace'},
    {'name': 'serial_no1', 'default': serial1,
     'description': 'choose left wrist camera by serial number'},
    {'name': 'serial_no2', 'default': serial2,
     'description': 'choose right wrist camera by serial number'},
    {'name': 'depth_module.depth_profile1', 'default': '640,480,30',
     'description': 'left wrist depth stream profile'},
    {'name': 'depth_module.depth_profile2', 'default': '640,480,30',
     'description': 'right wrist depth stream profile'},
    {'name': 'depth_module.color_profile1', 'default': '640,480,30',
     'description': 'left wrist color stream profile'},
    {'name': 'depth_module.color_profile2', 'default': '640,480,30',
     'description': 'right wrist color stream profile'},
    {'name': 'enable_depth1', 'default': 'true',
     'description': 'enable left wrist depth stream'},
    {'name': 'enable_depth2', 'default': 'true',
     'description': 'enable right wrist depth stream'},
    {'name': 'enable_color1', 'default': 'true',
     'description': 'enable left wrist color stream for local depth overlay blending'},
    {'name': 'enable_color2', 'default': 'true',
     'description': 'enable right wrist color stream for local depth overlay blending'},
    {'name': 'align_depth.enable1', 'default': 'false',
     'description': 'disable left depth alignment for low-latency teleoperation'},
    {'name': 'align_depth.enable2', 'default': 'false',
     'description': 'disable right depth alignment for low-latency teleoperation'},
    {'name': 'pointcloud.enable1', 'default': 'false',
     'description': 'disable left pointcloud generation for low-latency teleoperation'},
    {'name': 'pointcloud.enable2', 'default': 'false',
     'description': 'disable right pointcloud generation for low-latency teleoperation'},
    {'name': 'colorizer.enable1', 'default': 'false',
     'description': 'keep left RealSense colorizer disabled for teleoperation'},
    {'name': 'colorizer.enable2', 'default': 'false',
     'description': 'keep right RealSense colorizer disabled for teleoperation'},
]


def set_configurable_parameters(local_params):
    return {param['original_name']: LaunchConfiguration(param['name'])
            for param in local_params}


def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params


def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        rs_launch.declare_configurable_parameters(params2) +
        [
            GroupAction(
                condition=IfCondition(LaunchConfiguration('start_left_wrist')),
                actions=[
                    OpaqueFunction(
                        function=rs_launch.launch_setup,
                        kwargs={
                            'params': set_configurable_parameters(params1),
                            'param_name_suffix': '1',
                        },
                    ),
                ],
            ),
            GroupAction(
                condition=IfCondition(LaunchConfiguration('start_right_wrist')),
                actions=[
                    OpaqueFunction(
                        function=rs_launch.launch_setup,
                        kwargs={
                            'params': set_configurable_parameters(params2),
                            'param_name_suffix': '2',
                        },
                    ),
                ],
            ),
        ],
    )
