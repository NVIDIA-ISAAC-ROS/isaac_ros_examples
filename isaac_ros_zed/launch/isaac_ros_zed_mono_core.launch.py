# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os
from typing import Any, Dict

from ament_index_python import get_package_share_directory
from isaac_ros_examples import IsaacROSLaunchFragment
from launch.substitutions import Command
from launch_ros.actions import Node


class IsaacROSZedMonoLaunchFragment(IsaacROSLaunchFragment):

    @staticmethod
    def get_interface_specs() -> Dict[str, Any]:
        return {
            'camera_resolution': {'width': 1200, 'height': 720},
            'camera_frame': 'zed2_base_link',
            'camera_model': 'zed2',
            'focal_length': {
                # Approximation - most zed cameras should be close to this value
                'f_x': 380.0,
                # Approximation - most zed cameras should be close to this value
                'f_y': 380.0
            }
        }

    @staticmethod
    def get_launch_actions(interface_specs: Dict[str, Any]) -> Dict[str, Any]:
        # The zed camera mode name. zed, zed2, zed2i, zedm, zedx or zedxm
        camera_model = interface_specs['camera_model']

        # URDF/xacro file to be loaded by the Robot State Publisher node
        xacro_path = os.path.join(
            get_package_share_directory('zed_wrapper'),
            'urdf', 'zed_descr.urdf.xacro'
        )

        # ZED Configurations to be loaded by ZED Node
        config_common = os.path.join(
            get_package_share_directory('isaac_ros_zed'),
            'config',
            'zed_mono.yaml'
        )

        config_camera = os.path.join(
            get_package_share_directory('zed_wrapper'),
            'config',
            camera_model + '.yaml'
        )

        return {
            # Robot State Publisher node
            'rsp_node': Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='zed_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': Command(
                        [
                            'xacro', ' ', xacro_path, ' ',
                            'camera_name:=', camera_model, ' ',
                            'camera_model:=', camera_model
                        ])
                }]
            ),
            # ZED node using manual composition
            'zed_node': Node(
                package='zed_wrapper',
                executable='zed_wrapper',
                output='screen',
                parameters=[
                    config_common,  # Common parameters
                    config_camera,  # Camera related parameters
                ],
                arguments=[
                    '--ros-args',
                    '--remap', '/zed_node/rgb_raw/image_raw_color:=/image_raw',
                    '--remap', '/zed_node/rgb_raw/camera_info:=/camera_info'
                ]
            )
        }
