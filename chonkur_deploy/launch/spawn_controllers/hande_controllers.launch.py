#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    namespace = LaunchConfiguration("namespace")

    nodes = []

    nodes.append(spawn_controller("robotiq_gripper_hande_controller", namespace=namespace))
    nodes.append(spawn_controller("robotiq_activation_controller", namespace=namespace))

    return LaunchDescription(declared_arguments + nodes)
