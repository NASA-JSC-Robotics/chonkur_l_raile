#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
)
from chonkur_deploy.launch_helpers import spawn_controller


def generate_launch_description():

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
    enable_admittance = LaunchConfiguration("enable_admittance")
    namespace = LaunchConfiguration("namespace")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    nodes = []

    # active controllers
    nodes.append(spawn_controller("io_and_status_controller", namespace=namespace))
    nodes.append(spawn_controller("speed_scaling_state_broadcaster", namespace=namespace))
    nodes.append(spawn_controller("force_torque_sensor_broadcaster", namespace=namespace))
    nodes.append(spawn_controller("joint_trajectory_controller", namespace=namespace))

    # inactive controllers
    nodes.append(spawn_controller("scaled_joint_trajectory_controller", inactive=True, namespace=namespace))
    nodes.append(spawn_controller("forward_velocity_controller", inactive=True, namespace=namespace))
    nodes.append(spawn_controller("forward_position_controller", inactive=True, namespace=namespace))
    nodes.append(spawn_controller("freedrive_mode_controller", inactive=True, namespace=namespace))
    nodes.append(
        spawn_controller(
            "faked_forces_controller", inactive=True, namespace=namespace, condition=IfCondition(use_fake_hardware)
        )
    )

    # admittance
    admittance_controller_spawner = spawn_controller(
        "admittance_controller", inactive=True, namespace=namespace, condition=IfCondition(enable_admittance)
    )
    admittance_jtc_spawner = spawn_controller(
        "admittance_joint_trajectory_controller",
        inactive=True,
        namespace=namespace,
        condition=IfCondition(enable_admittance),
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
