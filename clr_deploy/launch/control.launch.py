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
from launch.conditions import UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
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
            "namespace",
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
            "use_fake_hardware",
            default_value="true",
            description="Start robot with simulated hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hande_dev_name",
            default_value="/dev/robotiq",
            description="File descriptor that will be generated for the tool communication device. "
            "The user has be be allowed to write to this location. ",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_package",
            default_value="clr_description",
            description="The package to find the robot description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_file",
            default_value="clr.urdf.xacro",
            description="The name of the robot description file. "
            "Must be in the 'urdf' folder of the description package.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If the robot is running in simulation, use the published clock",
        )
    )

    namespace = LaunchConfiguration("namespace")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    hande_dev_name = LaunchConfiguration("hande_dev_name")
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Main robot description for CLR. Additional arguments are available in the xacro, but we only
    # override a subset of those that change regularly depending on deployment.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(robot_description_package), "urdf", robot_description_file]),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "com_port_hande:=",
            hande_dev_name,
            " ",
        ]
    )

    robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}

    # State publisher for CLR. We include here for access to the top level robot_description content.
    # TODO: Separate this out once we are able to load the robot description from a topic in the controller manager.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # # start the controller manager node with all of the controller config files
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        # allow_substs allows tf_prefix to be pulled in
        parameters=[
            # TODO: Passing the robot description as a parameter is deprecated, but it is required for loading
            # the admittance controller because it needs the robot description to launch the KDL IK solver.
            # See https://tinyurl.com/3sf3u9ev.
            # Unfortunately, the description is loaded from a topic, the parameter is not passed to the controller and
            # it barfs on construction.
            robot_description,
            # CLR specific controllers
            parameter_file("clr_deploy", "controllers_common.yaml", True),
            parameter_file("clr_deploy", "clr_controllers.yaml", True),
            # Pulling in default controller configs for the hande, ur10e, lift, and rail
            parameter_file("chonkur_deploy", "ur10e_controllers.yaml", True),
            parameter_file("chonkur_deploy", "hande_controllers.yaml", True),
            parameter_file("ewellix_liftkit_deploy", "liftkit_controllers.yaml", True),
            parameter_file("vention_rail_deploy", "rail_controllers.yaml", True),
            {"use_sim_time": use_sim_time},
        ],
        output="both",
    )

    # Spawn all relevant CLR controllers
    spawn_controllers = include_launch_file(
        package_name="clr_deploy",
        launch_file="spawn_controllers.launch.py",
        launch_arguments={
            "namespace": namespace,
            "tf_prefix": tf_prefix,
            "use_fake_hardware": use_fake_hardware,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # CLR specific joint_state_broadcaster
    joint_state_broadcaster = spawn_controller("joint_state_broadcaster", namespace=namespace)

    # E-stop controller manager for CLR. We use the stopper for ChonkUR, since the rail and lift
    # do not require any consistent controllers. If that changes we may need to add a separate
    # stopper implementation for CLR.
    clr_controller_stopper = Node(
        package="chonkur_deploy",
        executable="chonkur_controller_stopper.py",
        parameters=[
            parameter_file("chonkur_deploy", "consistent_controllers.yaml", True),
            # This is generally the last controller to come up, so if it is available the controller
            # stopper should be good to initialize.
            {"target_controller": "admittance_joint_trajectory_controller"},
            {"use_sim_time": use_sim_time},
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    nodes = [robot_state_publisher_node, control_node, joint_state_broadcaster, clr_controller_stopper]
    return LaunchDescription(declared_arguments + nodes + [spawn_controllers])
