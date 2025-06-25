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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
)
from chonkur_deploy.launch_helpers import spawn_controller


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the hardware robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "is_sim",
            default_value="false",
            description="Whether or not the controllers are running in sim.",
        )
    )
    namespace = LaunchConfiguration("namespace")
    is_sim = LaunchConfiguration("is_sim")

    nodes = []

    # active controllers
    nodes.append(spawn_controller("force_torque_sensor_broadcaster", namespace=namespace))
    nodes.append(spawn_controller("joint_trajectory_controller", namespace=namespace))

    # Only include the ur_controllers in non-simulated environments
    nodes.append(
        spawn_controller("speed_scaling_state_broadcaster", namespace=namespace, condition=UnlessCondition(is_sim))
    )
    nodes.append(spawn_controller("io_and_status_controller", namespace=namespace, condition=UnlessCondition(is_sim)))

    # inactive controllers
    nodes.append(spawn_controller("scaled_joint_trajectory_controller", inactive=True, namespace=namespace))
    nodes.append(spawn_controller("forward_velocity_controller", inactive=True, namespace=namespace))
    nodes.append(spawn_controller("forward_position_controller", inactive=True, namespace=namespace))
    nodes.append(spawn_controller("freedrive_mode_controller", inactive=True, namespace=namespace))
    nodes.append(
        spawn_controller("faked_forces_controller", inactive=True, namespace=namespace, condition=IfCondition(is_sim))
    )

    # We always load the admittance controllers in an inactive state
    admittance_controller_spawner = spawn_controller(
        "admittance_controller",
        inactive=True,
        namespace=namespace,
    )
    admittance_jtc_spawner = spawn_controller(
        "admittance_joint_trajectory_controller",
        inactive=True,
        namespace=namespace,
    )

    # start the admittance jtc spawner after the admittance controller so that the jtc has the
    # right chained interfaces to hook into
    delay_admittance_jtc_spawner = RegisterEventHandler(
        OnProcessExit(target_action=admittance_controller_spawner, on_exit=[admittance_jtc_spawner])
    )
    admittance_controller_nodes = [
        admittance_controller_spawner,
        delay_admittance_jtc_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes + admittance_controller_nodes)
