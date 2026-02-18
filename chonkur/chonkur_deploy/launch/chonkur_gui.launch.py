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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from chonkur_deploy.launch_helpers import include_launch_file
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the robot.",
        )
    )

    # Initialize Arguments
    namespace = LaunchConfiguration("namespace")

    gui_config_path = os.path.join(get_package_share_directory("chonkur_deploy"), "config", "drt_ur_gui_config.yaml")

    ur_gui = include_launch_file(
        package_name="drt_ur_gui",
        launch_file="one_arm.launch.py",
        launch_arguments={
            "ns": namespace,
            "config_file_path": gui_config_path,
        }.items(),
    )

    nodes = [ur_gui]

    return LaunchDescription(declared_arguments + nodes)
