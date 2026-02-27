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
    clr_mujoco_description_file = "clr_xacro.urdf"

    mujoco_inputs = PathJoinSubstitution(
        [FindPackageShare(clr_mujoco_package_name), "description", "mujoco_inputs.xml"]
    )

    scene_xml = PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "description", "scene.xml"])

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

    # TODO: Generate this on the fly?
    make_mjcf_from_robot_description = Node(
        package="mujoco_ros2_control",
        executable="make_mjcf_from_robot_description.py",
        output="screen",
        arguments=[
            "--robot_description",
            robot_description_content,
            "--convert_stl_to_obj",
            "--save_only",
        ],
    )

    return LaunchDescription(
        [
            make_mjcf_from_robot_description,
        ]
    )
