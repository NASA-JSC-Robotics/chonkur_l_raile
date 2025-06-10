#!/usr/bin/env python3
#
# Copyright (c) 2025, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def spawn_controller(
    controller_name,
    inactive=False,
    controller_manager_name="controller_manager",
    timeout=300,
    namespace: LaunchConfiguration = "",
    condition=None,
):
    """
    Create a spawn controller node action for the specified controller and arguments.
    """
    inactive_flags = ["--inactive"] if inactive else []

    return Node(
        package="controller_manager",
        executable="spawner",
        name=controller_name,
        arguments=[
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            str(timeout),
            "--namespace",
            namespace,
            controller_name,
        ]
        + inactive_flags,
        output="both",
        condition=condition,
    )


def include_launch_file(package_name, launch_file, launch_arguments=None, condition=None):
    """
    Returns a launch description for the specified package name and launch file. The target file
    must be in the package's `launch/` directory.

    Additional launch arguments or conditions can be passed through as arguments.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), "launch", launch_file),
        ),
        launch_arguments=launch_arguments,
        condition=condition,
    )


def parameter_file(package, config, allow_substs=False):
    """
    Returns a ParameterFile for the specified package and config file name. The file must be in the
    package's `config/` directory.

    Specify `allow_substs` to pass variable substitutions to the yaml.
    """
    return ParameterFile(
        PathJoinSubstitution([get_package_share_directory(package), "config", config]),
        allow_substs=allow_substs,
    )
