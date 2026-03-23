from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import (
    FindPackageShare,
)
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_pregenerated_assets_dir",
            default_value="false",
            description="Use pre-generated assets dir. This is useful if you are just modifying an existing structure",
        )
    )

    clr_mujoco_package_name = "clr_mujoco_config"
    clr_mujoco_description_file = "clr_mujoco_xacro.urdf"

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
            " ",
            "include_scene_objects:=",
            "true",
        ]
    )

    default_arguments = [
        "--robot_description",
        robot_description_content,
        "--convert_stl_to_obj",
        "--save_only",
    ]

    args_with_assets_dir = default_arguments + [
        "--asset_dir",
        PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "description", "assets"]),
    ]

    # this version can be run for general
    make_mjcf_from_robot_description = Node(
        package="mujoco_ros2_control",
        executable="make_mjcf_from_robot_description.py",
        output="screen",
        arguments=default_arguments,
        condition=UnlessCondition(LaunchConfiguration("use_pregenerated_assets_dir")),
    )

    make_mjcf_from_robot_description_use_assets_dir = Node(
        package="mujoco_ros2_control",
        executable="make_mjcf_from_robot_description.py",
        output="screen",
        arguments=args_with_assets_dir,
        condition=IfCondition(LaunchConfiguration("use_pregenerated_assets_dir")),
    )

    return LaunchDescription(
        declared_arguments
        + [
            make_mjcf_from_robot_description,
            make_mjcf_from_robot_description_use_assets_dir,
        ]
    )
