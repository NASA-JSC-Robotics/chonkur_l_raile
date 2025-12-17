from launch import LaunchDescription
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

    clr_mujoco_package_name = "clr_mujoco_config"
    scene = PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "resources", "scene.xml"])
    mujoco_inputs = PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "description", "at_inputs.xml"])

    # Main robot description for CLR
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "urdf", "april_tag.urdf"]),
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
            "--save_only",
            "--scene",
            scene,
        ],
    )

    return LaunchDescription(
        [
            make_mjcf_from_robot_description,
        ]
    )
