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

from chonkur_deploy.launch_helpers import include_launch_file
from launch import LaunchDescription


def generate_launch_description():

    left_clr_moveit = include_launch_file(
        package_name="clr_moveit_config",
        launch_file="clr_moveit.launch.py",
        launch_arguments={
            "namespace": "left",
            "tf_prefix": "left_",
            "launch_rviz": "false",
        }.items(),
    )

    right_clr_moveit = include_launch_file(
        package_name="clr_moveit_config",
        launch_file="clr_moveit.launch.py",
        launch_arguments={
            "namespace": "right",
            "tf_prefix": "right_",
            "launch_rviz": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            left_clr_moveit,
            right_clr_moveit,
        ]
    )
