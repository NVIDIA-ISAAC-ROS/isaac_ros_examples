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

from typing import Any, Dict

from isaac_ros_examples import IsaacROSLaunchFragment
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


class IsaacROSUSBCameraLaunchFragment(IsaacROSLaunchFragment):

    @staticmethod
    def get_interface_specs() -> Dict[str, Any]:
        return {
            'camera_resolution': {'width': 640, 'height': 480}
        }

    @staticmethod
    def get_composable_nodes(interface_specs: Dict[str, Any]) -> Dict[str, ComposableNode]:
        return {
            'camera_node': ComposableNode(
                package='usb_cam',
                plugin='usb_cam::UsbCamNode',
                name='usb_cam',
                parameters=[{
                    'pixel_format': 'yuyv2rgb'
                }]
            )
        }


def generate_launch_description():
    usb_cam_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='usb_cam_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=IsaacROSUSBCameraLaunchFragment.get_composable_nodes(),
        output='screen'
    )

    return launch.LaunchDescription(
        [usb_cam_container] + IsaacROSUSBCameraLaunchFragment.get_launch_actions())
