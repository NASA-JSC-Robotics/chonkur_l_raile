import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    FindPackageShare,
    PathJoinSubstitution,
)


def generate_launch_description():

    clr_mujoco_package_name = "clr_mujoco_config"
    clr_mujoco_description_file = "clr_xacro.urdf"
    clr_mujoco_package_path = get_package_share_directory(clr_mujoco_package_name)

    mujoco_inputs = os.path.join(clr_mujoco_package_path, "description", "mujoco_inputs.xml")

    # Main robot description for CLR
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "urdf", clr_mujoco_description_file]),
            " ",
            # Grasp frames should not be converted to MJCF objects
            "add_grasp_push_frames:=",
            "false",
            " ",
            "model_env:=",
            "true",
        ]
    )

    make_mjcf_from_robot_description = Node(
        package="mujoco_ros2_simulation",
        executable="make_mjcf_from_robot_description.py",
        output="screen",
        arguments=[
            "-r",
            robot_description_content,
            "-m",
            mujoco_inputs,
            "-c",  # convert stl to obj
        ],
    )

    return LaunchDescription(
        [
            make_mjcf_from_robot_description,
        ]
    )
