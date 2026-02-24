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
from chonkur_deploy.launch_helpers import include_launch_file
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.substitutions import (
    FindPackageShare,
)


def generate_launch_description():

    declared_arguments = []

    clr_mujoco_package_name = "clr_mujoco_config"
    clr_mujoco_description_file = "clr_xacro.urdf"

    mjcf_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "urdf", clr_mujoco_description_file]),
            # Grasp frames should not be converted to MJCF objects
            " add_grasp_push_frames:=false",
            " model_env:=true",
        ]
    )

    generate_mjcf = Node(
        package="mujoco_ros2_control",
        executable="robot_description_to_mjcf.sh",
        output="both",
        emulate_tty=True,
        arguments=[
            "--publish_topic",
            "/mujoco_robot_description",
            "--robot_description",
            mjcf_robot_description_content,
            "--convert_stl_to_obj",
            "--asset_dir",
            # PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "description", "assets"]),
            "/home/er4-user/ws/src/mjcf_data/assets",
        ],
    )

    clr_launch = include_launch_file(
        package_name="clr_deploy",
        launch_file="control.launch.py",
        launch_arguments={
            "robot_description_package": "clr_mujoco_config",
            "robot_description_file": "clr_xacro.urdf",
            "model_env": "true",
            "use_fake_hardware": "false",
            "use_sim_time": "true",
            "is_sim": "true",
        }.items(),
    )

    point_cloud_proc = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("rgb/image_rect_color", "/wrist_mounted_camera/color/image_raw"),
            ("rgb/camera_info", "/wrist_mounted_camera/color/camera_info"),
            ("depth_registered/image_rect", "/wrist_mounted_camera/aligned_depth_to_color/image_raw"),
            ("points", "/wrist_mounted_camera/depth/color/points"),
        ],
    )

    return LaunchDescription(declared_arguments + [generate_mjcf, clr_launch, point_cloud_proc])
