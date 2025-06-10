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
from launch_ros.actions import Node
from chonkur_deploy.launch_helpers import include_launch_file


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.102",
            description="IP address by which the robot can be reached.",
        )
    )

    # Initialize Arguments
    headless_mode = LaunchConfiguration("headless_mode")
    robot_ip = LaunchConfiguration("robot_ip")

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        parameters=[
            {"headless_mode": headless_mode},
            {"robot_ip": robot_ip},
        ],
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
    )

    ur_dashboard_client = include_launch_file(
        package_name="ur_robot_driver",
        launch_file="ur_dashboard_client.launch.py",
        launch_arguments={
            "robot_ip": robot_ip,
        }.items(),
    )

    nodes = [robot_state_helper_node, urscript_interface, ur_dashboard_client]

    return LaunchDescription(declared_arguments + nodes)
