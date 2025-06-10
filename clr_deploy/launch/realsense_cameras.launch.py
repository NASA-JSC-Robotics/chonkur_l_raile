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

    lift_camera = include_launch_file(
        package_name="realsense2_camera",
        launch_file="rs_launch.py",
        launch_arguments={
            "camera_name": "lift_camera",
            "camera_namespace": "",
            "serial_no": "'207122078580'",
            "rgb_camera.profile": "1280,720,30",
            "initial_reset": "true",
            "pointcloud.enable": "false",
            "align_depth.enable": "true",
        }.items(),
    )

    wrist_camera = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="realsense_cameras.launch.py",
    )

    return LaunchDescription([lift_camera, wrist_camera])
