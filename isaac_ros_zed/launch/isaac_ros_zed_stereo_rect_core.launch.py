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
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


class IsaacROSZedStereoRectLaunchFragment(IsaacROSLaunchFragment):

    @staticmethod
    def get_interface_specs() -> Dict[str, Any]:
        return {
            'camera_resolution': {'width': 1200, 'height': 720},
            'camera_frame': 'zed2_base_link',
            'camera_model': 'zed2',
            'focal_length': {
                # Approximation - most zed cameras should be close to this value
                'f_x': 527.886474609375,
                # Approximation - most zed cameras should be close to this value
                'f_y': 527.886474609375
            }
        }

    @staticmethod
    def get_composable_nodes(interface_specs: Dict[str, Any]) -> Dict[str, ComposableNode]:
        # The zed camera mode name. zed, zed2, zed2i, zedm, zedx or zedxm
        camera_model = interface_specs['camera_model']
        pub_frame_rate = LaunchConfiguration('pub_frame_rate')

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
        base_parameters = [config_common, config_camera]
        override_parameters = {'general.pub_frame_rate': pub_frame_rate}
        parameters = base_parameters + [override_parameters]

        return {
            'image_format_converter_node_left': ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='image_format_node_left',
                parameters=[{
                    'encoding_desired': 'rgb8',
                    'image_width': interface_specs['camera_resolution']['width'],
                    'image_height': interface_specs['camera_resolution']['height'],
                }],
                remappings=[
                    ('image_raw', 'zed_node/left/image_rect_color'),
                    ('image', 'left/image_rect')]
            ),
            'image_format_converter_node_right': ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='image_format_node_right',
                parameters=[{
                    'encoding_desired': 'rgb8',
                    'image_width': interface_specs['camera_resolution']['width'],
                    'image_height': interface_specs['camera_resolution']['height'],
                }],
                remappings=[
                    ('image_raw', 'zed_node/right/image_rect_color'),
                    ('image', 'right/image_rect')]
            ),
            'tf_publisher': ComposableNode(
                name='static_transform_publisher',
                package='tf2_ros',
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                parameters=[{
                    'frame_id': 'base_link',
                    'child_frame_id': camera_model+'_camera_link',
                    'translation.x': 0.0,
                    'translation.y': 0.0,
                    'translation.z': 0.1,
                    'rotation.x': -0.5,
                    'rotation.y': 0.5,
                    'rotation.z': -0.5,
                    'rotation.w': 0.5}]
            ),
            'zed_wrapper_component': ComposableNode(
                package='zed_components',
                plugin='stereolabs::ZedCamera',
                name='zed_node',
                parameters=parameters,
                remappings=[
                    ('zed_node/left/camera_info', '/left/camera_info_rect'),
                    ('zed_node/right/camera_info', '/right/camera_info_rect'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
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

        return {
            'pub_frame_rate': DeclareLaunchArgument(
                'pub_frame_rate',  # Name of the argument
                default_value='15.0',  # Default value as a string
                description='Frame rate for publishing ZED camera data'
            ),
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
        }


def generate_launch_description():
    zed_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='zed_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=(
            IsaacROSZedStereoRectLaunchFragment.get_composable_nodes()
        ),
        output='screen'
    )

    return launch.LaunchDescription(
        [zed_container] + IsaacROSZedStereoRectLaunchFragment.get_launch_actions())
