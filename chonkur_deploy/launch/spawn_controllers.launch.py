#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from chonkur_deploy.launch_helpers import spawn_controller, include_launch_file


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_admittance",
            default_value="false",
            description="Allow the admittance controllers to spawn",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ns",
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
    # Initialize Arguments
    enable_admittance = LaunchConfiguration("enable_admittance")
    ns = LaunchConfiguration("ns")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    spawner_launch_args = {
        "enable_admittance": enable_admittance,
        "ns": ns,
        "use_fake_hardware": use_fake_hardware,
    }.items()

    ur10e_spawners = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="spawn_controllers/ur10e_controllers.launch.py",
        launch_arguments=spawner_launch_args,
    )
    gripper_spawners = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="spawn_controllers/hande_controllers.launch.py",
        launch_arguments=spawner_launch_args,
    )

    joint_state_broadcaster = spawn_controller("joint_state_broadcaster", namespace=ns)

    spawner_launch_files = [ur10e_spawners, gripper_spawners]

    return LaunchDescription(declared_arguments + spawner_launch_files + [joint_state_broadcaster])
