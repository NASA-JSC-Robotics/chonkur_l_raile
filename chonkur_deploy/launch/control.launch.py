#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from chonkur_deploy.launch_helpers import include_launch_file, spawn_controller


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

    # State publishers for ChonkUR
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

    # Spawn ChonkUR specific controllers
    spawn_controllers = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="spawn_controllers.launch.py",
        launch_arguments={
            "enable_admittance": enable_admittance,
            "ns": ns,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    # Include the joint state broadcaster at the top level control file so consumers
    # can add it as needed
    joint_state_broadcaster = spawn_controller("joint_state_broadcaster", namespace=ns)

    # Launch additional UR specific tools
    chonkur_tools = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="chonkur_tools.launch.py",
        launch_arguments={
            "headless_mode": headless_mode,
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    nodes = [control_node, joint_state_broadcaster]
    launch_files = [robot_state_publisher, spawn_controllers, chonkur_tools]
    return LaunchDescription(declared_arguments + nodes + launch_files)
