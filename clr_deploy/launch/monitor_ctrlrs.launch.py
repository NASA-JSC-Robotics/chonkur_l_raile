
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, FindExecutable

import os

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value='False',
            description='Set True if running sim',
        )
    )

    sim = LaunchConfiguration("sim")

    names = os.path.join(get_package_share_directory("clr_deploy"), 'config', 'clr_control_colors.yaml')

    sim_controller_check = ExecuteProcess(
        cmd=[[
            "gnome-terminal ", "-e ",
            '"',
            'ros2 run ',
            "drt_ros2_control_tools ",
            "controller_check.py ",
            "clr_deploy --sim --highlight ",
            names,
            '"'
        ]],
        shell=True,
        output='screen'
    )

    node = [sim_controller_check]

    return LaunchDescription(declared_arguments + node)
