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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os


def launch_setup(context, *args, **kwargs):

    sim_env = LaunchConfiguration("sim_env")

    if sim_env.perform(context) == "true":
        description_package = "clr_imetro_environments"
        description_file = "clr_trainer_multi_hatch.urdf.xacro"
        moveit_config_file_path = "srdf/clr_and_sim_mockups.srdf.xacro"
    else:
        description_package = "clr_description"
        description_file = "clr.urdf.xacro"
        moveit_config_file_path = "srdf/clr.srdf.xacro"

    launch_moveit = LaunchConfiguration("launch_moveit")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    description_full_path = os.path.join(get_package_share_directory(description_package), "urdf", description_file)
    moveit_config_package = "clr_moveit_config"

    # TODO: look into opaque function to pass in args to the robot description
    moveit_config = (
        MoveItConfigsBuilder("clr", package_name="clr_moveit_config")
        .robot_description(file_path=description_full_path)
        .robot_description_semantic(file_path=moveit_config_file_path)
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        condition=IfCondition(launch_moveit),
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.planning_scene_monitor,
            use_sim_time,
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="tf_prefix of the joint names, useful for \
        multi-robot setup. If changed, also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_moveit",
            default_value="true",
            description="Launch moveit?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch rviz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If the robot is running in simulation, use the published clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_env",
            default_value="false",
            description="Represent the iMETRO simulated mockup environment in the robot description and publish mockup joint states.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
