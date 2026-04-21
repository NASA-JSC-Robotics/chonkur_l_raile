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
from moveit_configs_utils import MoveItConfigsBuilder


def prefix_moveit_params(moveit_dict, tf_prefix):
    """Helper function to apply tf_prefixes to moveit config dicts on the fly

    MoveItConfigsBuilder loads joint_limits.yaml and OMPL planning configs
    via plain yaml_load, so we cannot substitute $(var tf_prefix) as we do
    elsewhere. This function patches the loaded dict after the fact, prepending
    tf_prefix to joint names wherever they appear as keys or inline references.
    """
    # No prefix == no-op
    if not tf_prefix:
        return moveit_dict

    # Right now we're only doing joint_limits and projection_evaluator. Since
    # those are the only two places we need to parameterize yaml.
    rdp = moveit_dict.get("robot_description_planning", {})
    if "joint_limits" in rdp:
        rdp["joint_limits"] = {
            (tf_prefix + name if isinstance(config, dict) else name): config
            for name, config in rdp["joint_limits"].items()
        }

    for group_config in moveit_dict.get("ompl", {}).values():
        if not isinstance(group_config, dict):
            continue
        pe = group_config.get("projection_evaluator", "")
        if pe.startswith("joints(") and pe.endswith(")"):
            joints = [tf_prefix + j.strip() for j in pe[7:-1].split(",")]
            group_config["projection_evaluator"] = f"joints({','.join(joints)})"

    return moveit_dict


def launch_setup(context, *args, **kwargs):

    mockups = LaunchConfiguration("include_mockups_in_description")
    namespace = LaunchConfiguration("namespace").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)

    if mockups.perform(context) == "true":
        moveit_config_file_path = "srdf/clr_and_sim_mockups.srdf.xacro"
    else:
        moveit_config_file_path = "srdf/clr.srdf.xacro"

    launch_moveit = LaunchConfiguration("launch_moveit")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    moveit_config_package = "clr_moveit_config"

    # Pull robot description from the topic
    moveit_config = (
        MoveItConfigsBuilder("clr", package_name="clr_moveit_config")
        .robot_description_semantic(
            file_path=moveit_config_file_path,
            mappings={"tf_prefix": tf_prefix},
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/clr_moveit_controllers.yaml")
        .to_moveit_configs()
    )
    moveit_params = prefix_moveit_params(moveit_config.to_dict(), tf_prefix)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="both",
        condition=IfCondition(launch_moveit),
        parameters=[
            use_sim_time,
            moveit_params,
            # Tell Ros2ControlManager plugin where the CM is
            {"ros_control_namespace": "/" + namespace if namespace else "/"},
            # Always subscribe to all joint states
            {"planning_scene_monitor_options.joint_states_topic": "joint_states"},
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
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the robot to control. " "For dual-robot setups, use 'left' or 'right'.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names. Must match the prefix "
            "used in the controller manager (e.g. 'left_' for dual setups).",
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
            "include_mockups_in_description",
            default_value="false",
            description="Represent the iMETRO mockup environment in the robot description.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
