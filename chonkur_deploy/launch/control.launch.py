#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.conditions import UnlessCondition
from chonkur_deploy.launch_helpers import include_launch_file


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
            "ns",
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
    ns = LaunchConfiguration("ns")
    robot_ip = LaunchConfiguration("robot_ip")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # # start the controller manager node with all of the controller config files
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        # allow_substs allows tf_prefix to be pulled in
        parameters=[
            ParameterFile(
                PathJoinSubstitution(
                    [get_package_share_directory("chonkur_deploy"), "config", "ur10e_update_rate.yaml"]
                ),
                allow_substs=True,
            ),
            ParameterFile(
                PathJoinSubstitution(
                    [get_package_share_directory("chonkur_deploy"), "config", "controllers_common.yaml"]
                ),
                allow_substs=True,
            ),
            ParameterFile(
                PathJoinSubstitution(
                    [get_package_share_directory("chonkur_deploy"), "config", "ur10e_controllers.yaml"]
                ),
                allow_substs=True,
            ),
            ParameterFile(
                PathJoinSubstitution(
                    [get_package_share_directory("chonkur_deploy"), "config", "hande_controllers.yaml"]
                ),
                allow_substs=True,
            ),
        ],
        remappings=[
            # remap to be able to use the global robot_description
            ("~/robot_description", "robot_description"),
        ],
        output="both",
    )

    robot_state_publisher = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="robot_state_publisher.launch.py",
        launch_arguments={
            "headless_mode": headless_mode,
            "ns": ns,
            "robot_ip": robot_ip,
            "tf_prefix": tf_prefix,
            "use_fake_hardware": use_fake_hardware,
            "hande_dev_name": "/dev/robotiq",
        }.items(),
    )

    spawn_controllers = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="spawn_controllers.launch.py",
        launch_arguments={
            "enable_admittance": enable_admittance,
            "ns": ns,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    ur_tools = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="ur_tools.launch.py",
        launch_arguments={
            "headless_mode": headless_mode,
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    chonkur_controller_stopper = Node(
        package="chonkur_deploy",
        executable="chonkur_controller_stopper.py",
        parameters=[
            ParameterFile(
                PathJoinSubstitution(
                    [get_package_share_directory("chonkur_deploy"), "config", "consistent_controllers.yaml"]
                ),
                allow_substs=True,
            ),
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    # wait for the controller stopper until everything else is loaded so that we can then manage,
    # instead of coming in during the middle of the loading process
    delay_controller_stopper = TimerAction(period=10.0, actions=[chonkur_controller_stopper])

    nodes = [control_node, chonkur_controller_stopper, delay_controller_stopper]
    launch_files = [robot_state_publisher, spawn_controllers, ur_tools]
    return LaunchDescription(declared_arguments + nodes + launch_files)
