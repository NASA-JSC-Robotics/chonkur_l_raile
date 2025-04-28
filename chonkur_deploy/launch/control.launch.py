#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
            "enable_admittance",
            default_value="false",
            description="Allow the admittance controllers to spawn",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the robot.",
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
            "tf_prefix",
            default_value="",
            description="tf prefix for the robot joints.",
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
    headless_mode = LaunchConfiguration("headless_mode")
    namespace = LaunchConfiguration("namespace")
    robot_ip = LaunchConfiguration("robot_ip")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # # start the controller manager node with all of the controller config files
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            parameter_file("chonkur_deploy", "controllers_common.yaml", True),
            parameter_file("chonkur_deploy", "ur10e_controllers.yaml", True),
            parameter_file("chonkur_deploy", "hande_controllers.yaml", True),
        ],
        remappings=[
            # remap to be able to use the global robot_description
            ("~/robot_description", "robot_description"),
        ],
        output="both",
    )

    # State publishers for ChonkUR
    robot_state_publisher = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="robot_state_publisher.launch.py",
        launch_arguments={
            "headless_mode": headless_mode,
            "namespace": namespace,
            "robot_ip": robot_ip,
            "tf_prefix": tf_prefix,
            "use_fake_hardware": use_fake_hardware,
            "hande_dev_name": "/dev/robotiq",
        }.items(),
    )

    # Spawn ChonkUR specific controllers
    spawn_controllers = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="spawn_controllers.launch.py",
        launch_arguments={
            "enable_admittance": enable_admittance,
            "namespace": namespace,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    # Include the joint state broadcaster at the top level control file so consumers
    # can add it as needed
    joint_state_broadcaster = spawn_controller("joint_state_broadcaster", namespace=namespace)

    # E-stop controller manager integration for ChonkUR
    chonkur_controller_stopper = Node(
        package="chonkur_deploy",
        executable="chonkur_controller_stopper.py",
        parameters=[
            parameter_file("chonkur_deploy", "consistent_controllers.yaml", True),
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    # wait for the controller stopper until everything else is loaded so that we can then manage,
    # instead of coming in during the middle of the loading process
    delay_controller_stopper = TimerAction(period=10.0, actions=[chonkur_controller_stopper])

    nodes = [control_node, joint_state_broadcaster]
    launch_files = [robot_state_publisher, spawn_controllers, delay_controller_stopper]
    return LaunchDescription(declared_arguments + nodes + launch_files)
