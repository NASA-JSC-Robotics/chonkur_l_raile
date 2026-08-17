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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = []

    trainer_hatch_mockup_config = (
        PathJoinSubstitution([FindPackageShare("clr_imetro_environments"), "config", "trainer_hatch_config.yaml"]),
    )
    nodes_to_start = [
        Node(
            package="hatch_4040",
            executable="mockup_state_manager.py",
            name="mockup_manager",
            output="both",
            parameters=[trainer_hatch_mockup_config],
        )
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
