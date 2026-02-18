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
    declared_arguments.append(
        DeclareLaunchArgument(
            "hande_dev_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device. "
            "The user has be be allowed to write to this location. ",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device.",
        )
    )

    # Initialize Arguments
    namespace = LaunchConfiguration("namespace")
    headless_mode = LaunchConfiguration("headless_mode")
    robot_ip = LaunchConfiguration("robot_ip")
    hande_dev_name = LaunchConfiguration("hande_dev_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="both",
        parameters=[
            {"headless_mode": headless_mode},
            {"robot_ip": robot_ip},
        ],
    )

    ur_dashboard_client = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="both",
        parameters=[{"robot_ip": robot_ip}],
    )

    hande_comm_node = Node(
        name="ur_tool_communication_hande",
        package="ur_robot_driver",
        executable="tool_communication.py",
        namespace=namespace,
        output="both",
        parameters=[
            {
                "robot_ip": robot_ip,
                "tcp_port": tool_tcp_port,
                "device_name": hande_dev_name,
            }
        ],
    )

    nodes = [robot_state_helper_node, ur_dashboard_client, hande_comm_node]

    return LaunchDescription(declared_arguments + nodes)
