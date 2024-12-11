# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
# flake8: noqa: F403,F405

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    use_rosbag_arg = DeclareLaunchArgument('use_rosbag', default_value='False',
                                           description='Whether to execute on rosbag')
    use_rosbag = LaunchConfiguration('use_rosbag')

    config_directory = get_package_share_directory('isaac_ros_multicamera_vo')
    foxglove_xml_config = os.path.join(config_directory, 'config', 'foxglove_bridge_launch.xml')
    foxglove_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([foxglove_xml_config])
    )

    urdf_file = os.path.join(config_directory, 'urdf', '2_realsense_calibration.urdf.xacro')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    rs_config_path = os.path.join(config_directory, 'config', 'multi_realsense.yaml')
    with open(rs_config_path) as rs_config_file:
        rs_config = yaml.safe_load(rs_config_file)

    remapping_list, optical_frames = [], []
    num_cameras = 2*len(rs_config['cameras'])

    for idx in range(num_cameras):
        infra_cnt = idx % 2+1
        camera_cnt = rs_config['cameras'][idx//2]['camera_name']
        optical_frames += [f'{camera_cnt}_infra{infra_cnt}_optical_frame']
        remapping_list += [(f'visual_slam/image_{idx}',
                            f'/{camera_cnt}/infra{infra_cnt}/image_rect_raw'),
                           (f'visual_slam/camera_info_{idx}',
                            f'/{camera_cnt}/infra{infra_cnt}/camera_info')]

    def realsense_capture(common_params, camera_params):
        stereo_capture = ComposableNode(
            name=camera_params['camera_name'],
            namespace='',
            package='realsense2_camera',
            plugin='realsense2_camera::RealSenseNodeFactory',
            parameters=[common_params | camera_params])
        return stereo_capture

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': False,
            'rectified_images': True,
            'enable_imu_fusion': False,
            'image_jitter_threshold_ms': 34.00,
            'base_frame': 'base_link',
            'enable_slam_visualization': False,
            'enable_landmarks_view': False,
            'enable_observations_view': False,
            'enable_ground_constraint_in_odometry': False,
            'enable_ground_constraint_in_slam': False,
            'enable_localization_n_mapping': True,
            'enable_debug_mode': False,
            'num_cameras': num_cameras,
            'min_num_images': num_cameras,
            'camera_optical_frames': optical_frames,
        }],
        remappings=remapping_list,
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=([visual_slam_node]),
        output='screen',
    )

    realsense_image_capture = LoadComposableNodes(
        target_container='visual_slam_launch_container',
        condition=LaunchConfigurationEquals('use_rosbag', 'False'),
        composable_node_descriptions=([realsense_capture(rs_config['common_params'], camera_config)
                                       for camera_config in rs_config['cameras']]),
    )

    state_publisher = Node(package='robot_state_publisher',
                           executable='robot_state_publisher',
                           output='both',
                           parameters=[{'robot_description': robot_description}])

    return LaunchDescription([use_rosbag_arg,
                              foxglove_bridge_launch,
                              state_publisher,
                              realsense_image_capture,
                              visual_slam_launch_container])
