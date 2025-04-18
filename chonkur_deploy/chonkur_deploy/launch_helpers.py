import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def spawn_controller(
    controller_name,
    inactive=False,
    controller_manager_name="controller_manager",
    timeout=300,
    namespace: LaunchConfiguration = "",
    condition=None,
):
    inactive_flags = ["--inactive"] if inactive else []

    return Node(
        package="controller_manager",
        executable="spawner",
        name=controller_name,
        arguments=[
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            str(timeout),
            "--namespace",
            namespace,
            controller_name,
        ]
        + inactive_flags,
        output="screen",
        condition=condition,
    )


def include_launch_file(package_name, launch_file, launch_arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), "launch", launch_file),
        ),
        launch_arguments=launch_arguments,
        condition=condition,
    )
