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
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    combined_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("clr_description"), "urdf", "dual_clrs.urdf.xacro"]),
        ]
    )

    combined_description = {"robot_description": ParameterValue(value=combined_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[combined_description],
    )

    left_clr = include_launch_file(
        package_name="clr_deploy",
        launch_file="control.launch.py",
        launch_arguments={
            "namespace": "left",
            "tf_prefix": "left_",
            "use_fake_hardware": "true",
            "is_sim": "true",
            "use_sim_time": "false",
        }.items(),
    )

    right_clr = include_launch_file(
        package_name="clr_deploy",
        launch_file="control.launch.py",
        launch_arguments={
            "namespace": "right",
            "tf_prefix": "right_",
            "use_fake_hardware": "true",
            "is_sim": "true",
            "use_sim_time": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            left_clr,
            right_clr,
        ]
    )
