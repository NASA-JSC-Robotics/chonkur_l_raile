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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hatch_4040",
            default_value="true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trainer",
            default_value="true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "second_trainer",
            default_value="false",
        )
    )

    nodes_to_start = [
        Node(
            package="hatch_4040",
            executable="hatch4040_manager.py",
            output="both",
            condition=IfCondition(LaunchConfiguration("hatch_4040")),
            parameters=[
                {"prefix": "lorge/"},
            ],
        ),
        Node(
            package="trainer",
            executable="trainer_manager.py",
            output="both",
            condition=IfCondition(LaunchConfiguration("trainer")),
            parameters=[
                {"prefix": LaunchConfiguration("tf_prefix")},
            ],
        ),
        Node(
            package="trainer",
            executable="trainer_manager.py",
            name="second_trainer_manager",
            output="both",
            condition=IfCondition(LaunchConfiguration("second_trainer")),
            parameters=[
                {"prefix": "second_trainer/"},
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
