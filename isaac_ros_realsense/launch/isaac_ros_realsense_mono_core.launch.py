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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


class IsaacROSRealSenseMonoLaunchFragment(IsaacROSLaunchFragment):

    @staticmethod
    def get_interface_specs() -> Dict[str, Any]:
        return {
            'camera_resolution': {'width': 640, 'height': 480},
            'camera_frame': 'camera_color_optical_frame',
            'focal_length': {
                # Approximation - most RealSense cameras should be close to this value
                'f_x': 380.0,
                # Approximation - most RealSense cameras should be close to this value
                'f_y': 380.0
            }
        }

    @staticmethod
    def get_composable_nodes(interface_specs: Dict[str, Any]) -> Dict[str, ComposableNode]:
        realsense_config_file_path = os.path.join(
            get_package_share_directory('isaac_ros_realsense'),
            'config', 'realsense_mono.yaml'
        )

        return {
            'camera_node': ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='realsense2_camera',
                namespace='',
                parameters=[
                    realsense_config_file_path
                ],
                remappings=[('color/image_raw', 'image_raw'),
                            ('color/camera_info', 'camera_info')]
            )
        }


def generate_launch_description():
    realsense_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='realsense_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=IsaacROSRealSenseMonoLaunchFragment.get_composable_nodes(),
        output='screen'
    )

    return launch.LaunchDescription(
        [realsense_container] + IsaacROSRealSenseMonoLaunchFragment.get_launch_actions())
