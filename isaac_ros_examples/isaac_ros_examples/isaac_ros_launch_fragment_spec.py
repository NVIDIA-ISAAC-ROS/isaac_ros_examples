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

from typing import Dict


class IsaacROSLaunchFragmentSpec:

    def __init__(self, package, class_name, filename=''):
        self.package = package
        self.class_name = class_name
        self.filename = filename if filename != '' else f'{package}_core.launch.py'


LAUNCH_FRAGMENT_SPECS: Dict[str, IsaacROSLaunchFragmentSpec] = {
    ################
    # Data Sources #
    ################

    # RealSense Camera
    'realsense_mono': IsaacROSLaunchFragmentSpec(
        'isaac_ros_realsense', 'IsaacROSRealSenseMonoLaunchFragment',
        'isaac_ros_realsense_mono_core.launch.py'),
    'realsense_mono_rect': IsaacROSLaunchFragmentSpec(
        'isaac_ros_realsense', 'IsaacROSRealSenseMonoRectLaunchFragment',
        'isaac_ros_realsense_mono_rect_core.launch.py'),
    'realsense_stereo_rect': IsaacROSLaunchFragmentSpec(
        'isaac_ros_realsense', 'IsaacROSRealSenseStereoRectLaunchFragment',
        'isaac_ros_realsense_stereo_rect_core.launch.py'),
    'realsense_stereo_rect_imu': IsaacROSLaunchFragmentSpec(
        'isaac_ros_realsense', 'IsaacROSRealSenseStereoRectImuLaunchFragment',
        'isaac_ros_realsense_stereo_rect_imu_core.launch.py'),
    'realsense_mono_rect_depth': IsaacROSLaunchFragmentSpec(
        'isaac_ros_realsense', 'IsaacROSRealSenseMonoRectDepthLaunchFragment',
        'isaac_ros_realsense_mono_rect_depth_core.launch.py'),

    # Argus Camera
    'argus_mono': IsaacROSLaunchFragmentSpec(
        'isaac_ros_argus_camera', 'IsaacROSArgusMonoLaunchFragment',
        'isaac_ros_argus_camera_mono_core.launch.py'),
    'argus_stereo': IsaacROSLaunchFragmentSpec(
        'isaac_ros_argus_camera', 'IsaacROSArgusStereoLaunchFragment',
        'isaac_ros_argus_camera_stereo_core.launch.py'),
    'argus_depth': IsaacROSLaunchFragmentSpec(
        'isaac_ros_argus_camera', 'IsaacROSArgusDepthLaunchFragment',
        'isaac_ros_argus_camera_depth_core.launch.py'),

    # USB Camera
    'usb_cam': IsaacROSLaunchFragmentSpec(
        'isaac_ros_usb_cam', 'IsaacROSUSBCameraLaunchFragment'),

    # ZED Camera
    'zed_mono_rect': IsaacROSLaunchFragmentSpec(
        'isaac_ros_zed', 'IsaacROSZedMonoRectLaunchFragment',
        'isaac_ros_zed_mono_rect_core.launch.py'),
    'zed_stereo_rect': IsaacROSLaunchFragmentSpec(
        'isaac_ros_zed', 'IsaacROSZedStereoRectLaunchFragment',
        'isaac_ros_zed_stereo_rect_core.launch.py'),
    'zed_mono': IsaacROSLaunchFragmentSpec(
        'isaac_ros_zed', 'IsaacROSZedMonoLaunchFragment',
        'isaac_ros_zed_mono_core.launch.py'),
    'zed_mono_rect_depth': IsaacROSLaunchFragmentSpec(
        'isaac_ros_zed', 'IsaacROSZedMonoRectDepthLaunchFragment',
        'isaac_ros_zed_mono_rect_depth_core.launch.py'),

    ###########################
    # Preprocessing Utilities #
    ###########################

    # Rectification
    'rectify_mono': IsaacROSLaunchFragmentSpec(
        'isaac_ros_image_proc', 'IsaacROSRectifyMonoLaunchFragment',
        'isaac_ros_image_rectify_mono_core.launch.py'),
    'rectify_stereo': IsaacROSLaunchFragmentSpec(
        'isaac_ros_image_proc', 'IsaacROSRectifyStereoLaunchFragment',
        'isaac_ros_image_rectify_stereo_core.launch.py'),

    # Resize + Rectification
    'resize_rectify_stereo': IsaacROSLaunchFragmentSpec(
        'isaac_ros_image_proc', 'IsaacROSResizeRectifyStereoLaunchFragment',
        'isaac_ros_image_resize_rectify_stereo_core.launch.py'),

    # Isaac ROS Depth Proc
    # Align depth to color
    'align_depth_to_color': IsaacROSLaunchFragmentSpec(
        'isaac_ros_depth_image_proc', 'IsaacROSAlignDepthToColorLaunchFragment',
        'isaac_ros_depth_image_proc_align_depth_to_color_core.launch.py'),
    # Convert depth to metric
    'convert_metric': IsaacROSLaunchFragmentSpec(
        'isaac_ros_depth_image_proc', 'IsaacROSConvertMetricLaunchFragment',
        'isaac_ros_depth_image_proc_convert_metric_core.launch.py'),
    # Convert depth to pointcloud
    'point_cloud_xyz': IsaacROSLaunchFragmentSpec(
        'isaac_ros_depth_image_proc', 'IsaacROSPointCloudXyzLaunchFragment',
        'isaac_ros_depth_image_proc_point_cloud_xyz_core.launch.py'),
    # Convert depth to colorized pointcloud
    'point_cloud_xyzrgb': IsaacROSLaunchFragmentSpec(
        'isaac_ros_depth_image_proc', 'IsaacROSPointCloudXyzrgbLaunchFragment',
        'isaac_ros_depth_image_proc_point_cloud_xyzrgb_core.launch.py'),

    # Isaac ROS Stereo Image Proc
    # SGM disparity estimation
    'disparity': IsaacROSLaunchFragmentSpec(
        'isaac_ros_stereo_image_proc', 'IsaacROSDisparityLaunchFragment',
        'isaac_ros_stereo_image_proc_disparity_core.launch.py'),
    # Convert disparity to colorized pointcloud
    'point_cloud': IsaacROSLaunchFragmentSpec(
        'isaac_ros_stereo_image_proc', 'IsaacROSPointCloudLaunchFragment',
        'isaac_ros_stereo_image_proc_point_cloud_core.launch.py'),
    # Convert disparity to depth
    'disparity_to_depth': IsaacROSLaunchFragmentSpec(
        'isaac_ros_stereo_image_proc', 'IsaacROSDisparityToDepthLaunchFragment',
        'isaac_ros_stereo_image_proc_disparity_to_depth_core.launch.py'),

    ###############
    # Core Graphs #
    ###############

    # Isaac ROS AprilTag
    'apriltag': IsaacROSLaunchFragmentSpec(
        'isaac_ros_apriltag', 'IsaacROSAprilTagLaunchFragment'),

    # Isaac ROS Compression
    'stereo_h264_decoder': IsaacROSLaunchFragmentSpec(
        'isaac_ros_h264_decoder', 'IsaacROSStereoH264DecoderLaunchFragment'),
    'stereo_h264_encoder': IsaacROSLaunchFragmentSpec(
        'isaac_ros_h264_encoder', 'IsaacROSStereoH264EncoderLaunchFragment'),


    # Isaac ROS DNN Stereo Depth
    'ess_disparity': IsaacROSLaunchFragmentSpec(
        'isaac_ros_ess', 'IsaacROSEssLaunchFragment'),
    'foundationstereo': IsaacROSLaunchFragmentSpec(
        'isaac_ros_foundationstereo', 'IsaacROSFoundationStereoLaunchFragment'),


    # Isaac ROS Image Pipeline
    'resize': IsaacROSLaunchFragmentSpec(
        'isaac_ros_image_proc', 'IsaacROSResizeLaunchFragment',
        'isaac_ros_image_resize_core.launch.py'),
    'flip': IsaacROSLaunchFragmentSpec(
        'isaac_ros_image_proc', 'IsaacROSFlipLaunchFragment',
        'isaac_ros_image_flip_core.launch.py'),
    'crop': IsaacROSLaunchFragmentSpec(
        'isaac_ros_image_proc', 'IsaacROSCropLaunchFragment',
        'isaac_ros_image_crop_core.launch.py'),
    'color_conversion': IsaacROSLaunchFragmentSpec(
        'isaac_ros_image_proc', 'IsaacROSColorConversionLaunchFragment',
        'isaac_ros_image_color_conversion_core.launch.py'),

    # Isaac ROS Image Segmentation
    'segformer': IsaacROSLaunchFragmentSpec(
        'isaac_ros_segformer', 'IsaacROSSegformerLaunchFragment'),
    'segment_anything': IsaacROSLaunchFragmentSpec(
        'isaac_ros_segment_anything', 'IsaacROSSegmentAnythingLaunchFragment'),
    'segment_anything2': IsaacROSLaunchFragmentSpec(
        'isaac_ros_segment_anything2', 'IsaacROSSegmentAnything2LaunchFragment'),
    'unet': IsaacROSLaunchFragmentSpec(
        'isaac_ros_unet', 'IsaacROSUNetLaunchFragment'),

    # Isaac ROS Object Detection
    'grounding_dino': IsaacROSLaunchFragmentSpec(
        'isaac_ros_grounding_dino', 'IsaacROSGroundingDinoLaunchFragment'),
    'rtdetr': IsaacROSLaunchFragmentSpec(
        'isaac_ros_rtdetr', 'IsaacROSRtDetrLaunchFragment'),
    'detectnet': IsaacROSLaunchFragmentSpec(
        'isaac_ros_detectnet', 'IsaacROSDetectnetLaunchFragment'),
    'yolov8': IsaacROSLaunchFragmentSpec(
        'isaac_ros_yolov8', 'IsaacROSYolov8LaunchFragment'),

    # Isaac ROS Pose Estimation
    'centerpose': IsaacROSLaunchFragmentSpec(
        'isaac_ros_centerpose', 'IsaacROSCenterPoseLaunchFragment'),
    'centerpose_visualizer': IsaacROSLaunchFragmentSpec(
        'isaac_ros_centerpose', 'IsaacROSCenterPoseVisualizerLaunchFragment',
        'isaac_ros_centerpose_visualizer_core.launch.py'),
    'dope': IsaacROSLaunchFragmentSpec(
        'isaac_ros_dope', 'IsaacROSDopeLaunchFragment'),
    'foundationpose': IsaacROSLaunchFragmentSpec(
        'isaac_ros_foundationpose', 'IsaacROSFoundationPoseLaunchFragment'),
    'foundationpose_tracking': IsaacROSLaunchFragmentSpec(
        'isaac_ros_foundationpose', 'IsaacROSFoundationPoseTrackingLaunchFragment',
        'isaac_ros_foundationpose_tracking_core.launch.py'),

    # Isaac ROS Visual Slam
    'visual_slam': IsaacROSLaunchFragmentSpec(
        'isaac_ros_visual_slam', 'IsaacROSVisualSlamLaunchFragment'),
}
