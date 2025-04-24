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

    # Load CLR specific controllers
    lift_rail_controller = spawn_controller("lift_rail_joint_trajectory_controller", namespace=ns, inactive=True)
    clr_controller = spawn_controller("clr_joint_trajectory_controller", namespace=ns, inactive=True)
    clr_servo_controller = spawn_controller("servo_controller", namespace=ns, inactive=True)
    streaming_controller = spawn_controller("streaming_controller", namespace=ns, inactive=True)

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
        launch_arguments={
            "enable_admittance": enable_admittance,
            "ns": ns,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )
    ewellix_controllers = include_launch_file(
        package_name="ewellix_liftkit_deploy",
        launch_file="spawn_controllers.launch.py",
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )
    vention_controllers = include_launch_file(
        package_name="vention_rail_deploy",
        launch_file="spawn_controllers.launch.py",
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    spawner_launch_files = [
        chonkur_controllers,
        ewellix_controllers,
        vention_controllers,
    ]

    return LaunchDescription(declared_arguments + controller_spawners + spawner_launch_files)
