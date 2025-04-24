#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from chonkur_deploy.launch_helpers import include_launch_file, parameter_file


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
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
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    headless_mode = LaunchConfiguration("headless_mode")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
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
        condition=UnlessCondition(use_fake_hardware),
    )

    ur_dashboard_client = include_launch_file(
        package_name="ur_robot_driver",
        launch_file="ur_dashboard_client.launch.py",
        launch_arguments={
            "robot_ip": robot_ip,
        }.items(),
        condition=UnlessCondition(use_fake_hardware),
    )

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

    nodes = [robot_state_helper_node, urscript_interface, ur_dashboard_client, delay_controller_stopper]

    return LaunchDescription(declared_arguments + nodes)
