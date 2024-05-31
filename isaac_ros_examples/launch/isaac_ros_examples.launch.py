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

import importlib.util
import json
import os

from ament_index_python.packages import get_package_share_directory
from isaac_ros_examples import IsaacROSLaunchFragment, IsaacROSLaunchFragmentSpec, \
    LAUNCH_FRAGMENT_SPECS
import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
import rclpy


def load_launch_fragment(fragment_spec: IsaacROSLaunchFragmentSpec) -> IsaacROSLaunchFragment:
    """
    Load a Launch Fragment as a Python class from a specific file in a different ROS package.

    Parameters
    ----------
    fragment_spec : IsaacROSLaunchFragmentSpec
        Launch Fragment specification that indicates the package, filename, and class name to load

    Returns
    -------
    IsaacROSLaunchFragment
        The loaded Launch Fragment class

    """
    path = os.path.join(
        get_package_share_directory(fragment_spec.package),
        'launch',
        fragment_spec.filename
    )
    spec = importlib.util.spec_from_file_location(f'{fragment_spec.package}.module', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, fragment_spec.class_name)


def parse_launch_fragments(context):
    """
    Parse the launch fragments specified by the user and prepare the composite launch graph.

    Parameters
    ----------
    context : LaunchContext
        The launch context, containing all arguments passed in by the user

    Returns
    -------
    List[LaunchDescriptionEntity]
        List of launch actions, including the composite node container and all composable nodes

    """
    logger = rclpy.logging.get_logger('isaac_ros_examples')

    launch_fragments_str: str = \
        context.perform_substitution(LaunchConfiguration('launch_fragments'))

    if launch_fragments_str == '':
        logger.warning('No launch fragments specified.')
        return []

    # Parse string into fragments list
    fragments = []
    for fragment_key in launch_fragments_str.split(','):
        if not fragment_key:
            continue

        assert fragment_key in LAUNCH_FRAGMENT_SPECS, \
            f'Invalid launch fragment key: {fragment_key}'

        fragment_spec = LAUNCH_FRAGMENT_SPECS[fragment_key]
        fragment = load_launch_fragment(fragment_spec)
        fragments.append(fragment)

    # Collect interface specs from all fragments
    interface_specs = {}

    for fragment in fragments:
        for key, value in fragment.get_interface_specs().items():
            if key in interface_specs:
                logger.warning(
                    f'Warning: Interface spec "{key}" is specified multiple times. '
                    'The last value will be used'
                )
            interface_specs[key] = value

    # If interface specs file is provided, override specs with values from file
    interface_specs_file: str = \
        context.perform_substitution(LaunchConfiguration('interface_specs_file'))

    if interface_specs_file:
        with open(interface_specs_file, 'r') as file:
            override_specs = json.load(file)
            for key, value in override_specs.items():
                if key in interface_specs:
                    logger.info(
                        f'Info: Interface spec "{key}" is overridden. '
                        'The value from the overriding file will be used'
                    )
                interface_specs[key] = value

    # Collect composable nodes and launch actions from all fragments
    composable_nodes = []
    launch_actions = []
    for fragment in fragments:
        composable_nodes.extend(fragment.get_composable_nodes(interface_specs).values())
        launch_actions.extend(fragment.get_launch_actions(interface_specs).values())

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='container',
        namespace='isaac_ros_examples',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    return launch_actions + [container]


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(
            'launch_fragments',
            default_value='',
            description='Comma separated list of fragment keys to launch',
        ),
        DeclareLaunchArgument(
            'interface_specs_file',
            default_value='',
            description='Path to JSON file containing manually-specified interface specs',
        )
    ]

    return launch.LaunchDescription(
        launch_args + [OpaqueFunction(function=parse_launch_fragments)])
