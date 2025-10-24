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
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

WIDTH = 1280
HEIGHT = 720


class IsaacROSRealSenseStereoRectLaunchFragment(IsaacROSLaunchFragment):

    @staticmethod
    def get_interface_specs() -> Dict[str, Any]:
        return {
            'camera_resolution': {'width': WIDTH, 'height': HEIGHT},
            'camera_frame': 'camera_infra1_optical_frame',
            'focal_length': {
                # Approximation - most RealSense cameras should be close to this value
                'f_x': 635.0,
                # Approximation - most RealSense cameras should be close to this value
                'f_y': 635.0
            }
        }

    @staticmethod
    def get_composable_nodes(interface_specs: Dict[str, Any]) -> Dict[str, ComposableNode]:
        realsense_config_file = LaunchConfiguration('realsense_config_file')
        camera_name = 'camera'
        return {
            'camera_node': ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name=camera_name,
                namespace='',
                parameters=[
                    realsense_config_file
                ],
                remappings=[(f'/{camera_name}/infra1/image_rect_raw',
                             'infra1/image_rect_raw_mono'),
                            (f'/{camera_name}/infra1/camera_info', 'left/camera_info_rect'),
                            (f'/{camera_name}/infra2/image_rect_raw',
                             'infra2/image_rect_raw_mono'),
                            (f'/{camera_name}/infra2/camera_info', 'right/camera_info_rect')]
            ),
            'image_format_converter_left_node': ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='image_format_left',
                parameters=[{
                        'encoding_desired': 'rgb8',
                        'image_width': WIDTH,
                        'image_height': HEIGHT
                }],
                remappings=[
                    ('image_raw', 'infra1/image_rect_raw_mono'),
                    ('image', 'left/image_rect')]
            ),
            'image_format_converter_right_node': ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='image_format_right',
                parameters=[{
                        'encoding_desired': 'rgb8',
                        'image_width': WIDTH,
                        'image_height': HEIGHT
                }],
                remappings=[
                    ('image_raw', 'infra2/image_rect_raw_mono'),
                    ('image', 'right/image_rect')]
            )
        }

    @staticmethod
    def get_launch_actions(interface_specs: Dict[str, Any]) -> \
            Dict[str, launch.actions.OpaqueFunction]:
        return {
            'realsense_config_file': DeclareLaunchArgument(
                'realsense_config_file',
                default_value=os.path.join(
                    get_package_share_directory('isaac_ros_realsense'),
                    'config', 'realsense_stereo.yaml'
                )
            ),
        }


def generate_launch_description():
    realsense_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='realsense_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=(
            IsaacROSRealSenseStereoRectLaunchFragment.get_composable_nodes()
        ),
        output='screen'
    )

    return launch.LaunchDescription(
        [realsense_container] + IsaacROSRealSenseStereoRectLaunchFragment.get_launch_actions())
