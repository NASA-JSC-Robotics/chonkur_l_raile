#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from chonkur_deploy.launch_helpers import (
    include_launch_file,
    parameter_file,
    spawn_controller,
)


def generate_launch_description():

    # arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ns",
            default_value="",
            description="Namespace for the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf prefix for the robot joints.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_admittance",
            default_value="false",
            description="Allow the admittance controllers to spawn",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with simulated hardware mirroring command to its states.",
        )
    )

    enable_admittance = LaunchConfiguration("enable_admittance")
    ns = LaunchConfiguration("ns")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # # start the controller manager node with all of the controller config files
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        # allow_substs allows tf_prefix to be pulled in
        parameters=[
            # CLR specific controllers
            parameter_file("clr_deploy", "controllers_common.yaml", True),
            parameter_file("clr_deploy", "clr_controllers.yaml", True),
            # Pulling in default controller configs for the hande, ur10e, lift, and rail
            parameter_file("chonkur_deploy", "ur10e_update_rate.yaml", True),
            parameter_file("chonkur_deploy", "ur10e_controllers.yaml", True),
            parameter_file("chonkur_deploy", "hande_controllers.yaml", True),
            parameter_file("ewellix_liftkit_deploy", "liftkit_controllers.yaml", True),
            parameter_file("vention_rail_deploy", "rail_controllers.yaml", True),
        ],
        remappings=[
            # remap to be able to use the global robot_description
            ("~/robot_description", "robot_description"),
        ],
        output="both",
    )

    robot_state_publisher = include_launch_file(
        package_name="clr_deploy",
        launch_file="robot_state_publisher.launch.py",
        launch_arguments={
            "ns": ns,
            "tf_prefix": tf_prefix,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    # Spawn all relevant CLR controllers
    spawn_controllers = include_launch_file(
        package_name="clr_deploy",
        launch_file="spawn_controllers.launch.py",
        launch_arguments={
            "ns": ns,
            "tf_prefix": tf_prefix,
            "use_fake_hardware": use_fake_hardware,
            "enable_admittance": enable_admittance,
        }.items(),
    )

    # Additional ROS node utilities for ChonkUR
    ur_tools = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="chonkur_tools.launch.py",
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    # CLR specific joint_state_broadcaster
    joint_state_broadcaster = spawn_controller("joint_state_broadcaster", namespace=ns)

    # E-stop controller manager for CLR
    clr_estop_safety = Node(
        package="clr_control",
        executable="estop_safety.py",
        condition=UnlessCondition(use_fake_hardware),
    )

    # wait for the controller stopper until everything else is loaded so that we can then manage,
    # instead of coming in during the middle of the loading process
    delay_clr_estop_safety = TimerAction(period=10.0, actions=[clr_estop_safety])

    nodes = [control_node, joint_state_broadcaster, delay_clr_estop_safety]
    launch_files = [robot_state_publisher, spawn_controllers, ur_tools]
    return LaunchDescription(declared_arguments + nodes + launch_files)
