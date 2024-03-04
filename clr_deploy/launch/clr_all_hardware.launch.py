from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


# Launch Controllers and Cameras
def generate_launch_description():

    declared_arguments = []
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
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="start rviz?",
        )
    )    

    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")    
    rviz = LaunchConfiguration("rviz")

    launch_nodes = []

    clr_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("clr_deploy"), 'launch','clr_control.launch.py')),
        launch_arguments={
            "tf_prefix": tf_prefix,
            "use_fake_hardware": use_fake_hardware,
            "headless_mode": headless_mode,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "rviz": rviz,
        }.items(),
    )
    launch_nodes.append(clr_controllers_launch)

    if(use_fake_hardware == "false"):
        clr_cameras = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("clr_deploy"), 'launch','realsense_cameras.launch.py')),
        )

        launch_nodes.append(clr_cameras)

    return LaunchDescription(declared_arguments + launch_nodes)
