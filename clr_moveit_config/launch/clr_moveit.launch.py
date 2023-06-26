
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from ur_moveit_config.launch_common import load_yaml
from moveit_configs_utils import MoveItConfigsBuilder

import os

def generate_launch_description():

    description_package = 'clr_description'
    description_file = 'clr.urdf.xacro'
    description_full_path = os.path.join(get_package_share_directory(description_package), 'urdf',description_file)
    moveit_config_package = 'clr_moveit_config'
    moveit_config_srdf_file = 'clr.srdf'


    moveit_config = (
        MoveItConfigsBuilder("clr", package_name="clr_moveit_config")
        .robot_description(file_path=description_full_path)
        .robot_description_semantic(file_path="srdf/clr.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

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

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    tf_prefix = LaunchConfiguration("tf_prefix")
    launch_moveit = LaunchConfiguration("launch_moveit")

    print(moveit_config.robot_description_kinematics)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        condition=IfCondition(launch_moveit),
        parameters=[
            moveit_config.to_dict(),         
            ]
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.planning_scene_monitor,
        ],
    )    


    # TODO: turn move group and rviz node as optional
    nodes_to_start = [
        move_group_node, 
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)    
