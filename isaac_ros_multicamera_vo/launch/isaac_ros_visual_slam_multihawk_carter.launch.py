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

import os

from ament_index_python.packages import get_package_share_directory
import isaac_ros_launch_utils as lu
import launch
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.actions import SetParameter, SetRemap
from launch_ros.descriptions import ComposableNode
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


MODULE_IDS = {
    # Nova Carter Hawk ids
    'front_stereo_camera': 5,
    'back_stereo_camera': 6,
    'left_stereo_camera': 7,
    'right_stereo_camera': 2
}


def hawk_capture(camera_name):
    stereo_capture = ComposableNode(
            name=f'{camera_name}_node',
            package='isaac_ros_hawk',
            plugin='nvidia::isaac_ros::hawk::HawkNode',
            namespace=f'{camera_name}',
            parameters=[{'module_id': MODULE_IDS[camera_name],
                         'camera_link_frame_name': f'{camera_name}',
                         'left_camera_frame_name': f'{camera_name}_left',
                         'right_camera_frame_name': f'{camera_name}_right'}],
            remappings=[
                (f'/{camera_name}/correlated_timestamp', '/correlated_timestamp'),
            ],
        )
    return stereo_capture


def hawk_decoder(name, identifier):

    decoder = ComposableNode(
        name=f'{name}_{identifier}_decoder_node',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        namespace=f'{name}_stereo_camera/{identifier}',
        remappings=[('image_uncompressed', 'image_raw')],
    )
    return decoder


def hawk_processing(name, identifier):
    rectify_node = ComposableNode(
        name=f'{name}_{identifier}_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        namespace=f'{name}_stereo_camera/{identifier}',
        parameters=[{
            'output_width': 1920,
            'output_height': 1200,
            'type_negotiation_duration_s': 1,
        }],
    )
    return rectify_node


def generate_launch_description():

    use_rectify_arg = DeclareLaunchArgument('use_rectify', default_value='False',
                                            description='Whether to use rectify nodes')
    use_rectify = IfCondition(LaunchConfiguration('use_rectify', default='False'))

    use_rosbag_arg = DeclareLaunchArgument('use_rosbag', default_value='False',
                                           description='Whether to execute on rosbag')
    use_rosbag = IfCondition(LaunchConfiguration('use_rosbag', default='False'))

    nova_carter_description = lu.include(
            'nova_carter_description',
            'launch/nova_carter_description.launch.py',
        )

    correlated_timestamp_driver_node = ComposableNode(
        package='isaac_ros_correlated_timestamp_driver',
        plugin='nvidia::isaac_ros::correlated_timestamp_driver::CorrelatedTimestampDriverNode',
        name='correlated_timestamp_driver',
        parameters=[{'use_time_since_epoch': False,
                     'nvpps_dev_name': '/dev/nvpps0'}])

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{

            # general params
            'use_sim_time': False,
            'override_publishing_stamp': False,
            'enable_ground_constraint_in_odometry': True,
            'enable_localization_n_mapping': True,

            # frame params
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,

            # camera optical frames should have the same order as camera topics
            'input_camera_optical_frames': [
                'front_stereo_camera_left_optical',
                'front_stereo_camera_right_optical',
                'back_stereo_camera_left_optical',
                'back_stereo_camera_right_optical',
                'left_stereo_camera_left_optical',
                'left_stereo_camera_right_optical',
                'right_stereo_camera_left_optical',
                'right_stereo_camera_right_optical',
            ],

            # camera params
            'img_jitter_threshold_ms': 34.0,
            'denoise_input_images': False,
            'rectified_images': False,

            # multicamera params
            'num_cameras': 8,
            'min_num_images': 4,
            'sync_matching_threshold_ms': 5.0,

            # inertial odometry is not supported for multi-camera visual odometry
            'enable_imu_fusion': False,

            # visualization params
            'path_max_size': 1024,
            'enable_slam_visualization': False,
            'enable_landmarks_view': False,
            'enable_observations_view': False,

            # debug params
            'verbosity': 0,
            'enable_debug_mode': False,
            'debug_dump_path': '/tmp/cuvslam',
        }],
        remappings=[
            ('visual_slam/image_0', '/front_stereo_camera/left/image_raw'),
            ('visual_slam/camera_info_0', '/front_stereo_camera/left/camera_info'),
            ('visual_slam/image_1', '/front_stereo_camera/right/image_raw'),
            ('visual_slam/camera_info_1', '/front_stereo_camera/right/camera_info'),
            ('visual_slam/image_2', '/back_stereo_camera/left/image_raw'),
            ('visual_slam/camera_info_2', '/back_stereo_camera/left/camera_info'),
            ('visual_slam/image_3', '/back_stereo_camera/right/image_raw'),
            ('visual_slam/camera_info_3', '/back_stereo_camera/right/camera_info'),
            ('visual_slam/image_4', '/left_stereo_camera/left/image_raw'),
            ('visual_slam/camera_info_4', '/left_stereo_camera/left/camera_info'),
            ('visual_slam/image_5', '/left_stereo_camera/right/image_raw'),
            ('visual_slam/camera_info_5', '/left_stereo_camera/right/camera_info'),
            ('visual_slam/image_6', '/right_stereo_camera/left/image_raw'),
            ('visual_slam/camera_info_6', '/right_stereo_camera/left/camera_info'),
            ('visual_slam/image_7', '/right_stereo_camera/right/image_raw'),
            ('visual_slam/camera_info_7', '/right_stereo_camera/right/camera_info')
            ]
    )

    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=([visual_slam_node]),
        output='screen',
        condition=LaunchConfigurationEquals('use_rectify', 'False')
    )

    visual_slam_rect_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=([
            hawk_processing('front', 'left'),
            hawk_processing('front', 'right'),
            hawk_processing('back', 'left'),
            hawk_processing('back', 'right'),
            hawk_processing('left', 'left'),
            hawk_processing('left', 'right'),
            hawk_processing('right', 'left'),
            hawk_processing('right', 'right'),
            visual_slam_node
        ]),
        output='screen',
        condition=LaunchConfigurationEquals('use_rectify', 'True')
    )

    hawk_h264_decoders = LoadComposableNodes(
        target_container='visual_slam_launch_container',
        condition=use_rosbag,
        composable_node_descriptions=[
            hawk_decoder('front', 'left'),
            hawk_decoder('front', 'right'),
            hawk_decoder('back', 'left'),
            hawk_decoder('back', 'right'),
            hawk_decoder('left', 'left'),
            hawk_decoder('left', 'right'),
            hawk_decoder('right', 'left'),
            hawk_decoder('right', 'right'),
        ]
    )

    hawk_image_capture = LoadComposableNodes(
        target_container='visual_slam_launch_container',
        condition=LaunchConfigurationEquals('use_rosbag', 'False'),
        composable_node_descriptions=[
            hawk_capture('front_stereo_camera'),
            hawk_capture('back_stereo_camera'),
            hawk_capture('left_stereo_camera'),
            hawk_capture('right_stereo_camera'),
            correlated_timestamp_driver_node,
        ]
    )

    group_action = GroupAction([
        use_rectify_arg,
        use_rosbag_arg,

        SetParameter(name='rectified_images', value=True,
                     condition=LaunchConfigurationEquals('use_rectify', 'True')),

        SetRemap(src=['visual_slam/image_0'],
                 dst=['/front_stereo_camera/left/image_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/camera_info_0'],
                 dst=['/front_stereo_camera/left/camera_info_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/image_1'],
                 dst=['/front_stereo_camera/right/image_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/camera_info_1'],
                 dst=['/front_stereo_camera/right/camera_info_rect'],
                 condition=use_rectify),

        SetRemap(src=['visual_slam/image_2'],
                 dst=['/back_stereo_camera/left/image_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/camera_info_2'],
                 dst=['/back_stereo_camera/left/camera_info_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/image_3'],
                 dst=['/back_stereo_camera/right/image_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/camera_info_3'],
                 dst=['/back_stereo_camera/right/camera_info_rect'],
                 condition=use_rectify),

        SetRemap(src=['visual_slam/image_4'],
                 dst=['/left_stereo_camera/left/image_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/camera_info_4'],
                 dst=['/left_stereo_camera/left/camera_info_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/image_5'],
                 dst=['/left_stereo_camera/right/image_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/camera_info_5'],
                 dst=['/left_stereo_camera/right/camera_info_rect'],
                 condition=use_rectify),

        SetRemap(src=['visual_slam/image_6'],
                 dst=['/right_stereo_camera/left/image_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/camera_info_6'],
                 dst=['/right_stereo_camera/left/camera_info_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/image_7'],
                 dst=['/right_stereo_camera/right/image_rect'],
                 condition=use_rectify),
        SetRemap(src=['visual_slam/camera_info_7'],
                 dst=['/right_stereo_camera/right/camera_info_rect'],
                 condition=use_rectify),
        visual_slam_container,
        visual_slam_rect_container
    ])

    config_directory = get_package_share_directory('isaac_ros_multicamera_vo')
    foxglove_xml_config = os.path.join(config_directory, 'config', 'foxglove_bridge_launch.xml')
    foxglove_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([foxglove_xml_config])
    )

    return launch.LaunchDescription([nova_carter_description,
                                     foxglove_bridge_launch,
                                     hawk_image_capture,
                                     group_action,
                                     hawk_h264_decoders])
