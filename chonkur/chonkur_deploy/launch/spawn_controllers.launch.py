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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from chonkur_deploy.launch_helpers import include_launch_file


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_admittance",
            default_value="false",
            description="Allow the admittance controllers to spawn",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the hardware robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    # Initialize Arguments
    enable_admittance = LaunchConfiguration("enable_admittance")
    namespace = LaunchConfiguration("namespace")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    spawner_launch_args = {
        "enable_admittance": enable_admittance,
        "namespace": namespace,
        "use_fake_hardware": use_fake_hardware,
    }.items()

    ur10e_spawners = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="spawn_controllers/ur10e_controllers.launch.py",
        launch_arguments=spawner_launch_args,
    )
    gripper_spawners = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="spawn_controllers/hande_controllers.launch.py",
        launch_arguments=spawner_launch_args,
    )

    spawner_launch_files = [ur10e_spawners, gripper_spawners]

    return LaunchDescription(declared_arguments + spawner_launch_files)
