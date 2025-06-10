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
from chonkur_deploy.launch_helpers import include_launch_file


def generate_launch_description():

    clr_launch = include_launch_file(
        package_name="clr_deploy",
        launch_file="control.launch.py",
        launch_arguments={
            "use_fake_hardware": "false",
            # The admittance controller will not work with the default humble controller, as the default IK
            # solver will include the full kinematic chain from the tool to the rail - which is 2 extra joints.
            # This is "fixed" moving forward by https://github.com/ros-controls/kinematics_interface/pull/73/files.
            # "enable_admittance": "true",
        }.items(),
    )

    camera_launch = include_launch_file(
        package_name="clr_deploy",
        launch_file="realsense_cameras.launch.py",
    )

    return LaunchDescription([clr_launch, camera_launch])
