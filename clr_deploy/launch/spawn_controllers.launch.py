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
from launch.substitutions import LaunchConfiguration, OrSubstitution
from chonkur_deploy.launch_helpers import spawn_controller, include_launch_file


def generate_launch_description():
    # Declare arguments
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
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If the robot is running in simulation, use the published clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "is_sim",
            default_value="false",
            description="If the robot is running with simulated drivers in some capacity (e.g. mujoco).",
        )
    )

    # Initialize Arguments
    namespace = LaunchConfiguration("namespace")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    is_sim = LaunchConfiguration("is_sim")

    spawner_launch_args = {
        "namespace": namespace,
        "use_fake_hardware": use_fake_hardware,
        "use_sim_time": use_sim_time,
        "is_sim": is_sim,
    }.items()

    # Load CLR specific controllers
    lift_rail_controller = spawn_controller("lift_rail_joint_trajectory_controller", namespace=namespace, inactive=True)
    clr_controller = spawn_controller("clr_joint_trajectory_controller", namespace=namespace, inactive=True)
    clr_servo_controller = spawn_controller("servo_controller", namespace=namespace, inactive=True)
    streaming_controller = spawn_controller("streaming_controller", namespace=namespace, inactive=True)

    controller_spawners = [
        lift_rail_controller,
        clr_controller,
        clr_servo_controller,
        streaming_controller,
    ]

    # Pull in additional spawners for Chonkur, Ewellix Lift, and Vention Rail
    chonkur_controllers = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="spawn_controllers.launch.py",
        launch_arguments=spawner_launch_args,
    )
    ewellix_controllers = include_launch_file(
        package_name="ewellix_liftkit_deploy",
        launch_file="spawn_controllers.launch.py",
        launch_arguments=spawner_launch_args,
    )
    vention_controllers = include_launch_file(
        package_name="vention_rail_deploy",
        launch_file="spawn_controllers.launch.py",
        launch_arguments={
            # Override this directly to handle the rail e stop controller
            "use_fake_hardware": OrSubstitution(use_fake_hardware, is_sim)
        }.items(),
    )

    spawner_launch_files = [
        chonkur_controllers,
        ewellix_controllers,
        vention_controllers,
    ]

    return LaunchDescription(declared_arguments + controller_spawners + spawner_launch_files)
