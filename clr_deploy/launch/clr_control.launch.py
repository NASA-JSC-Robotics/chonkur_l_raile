from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os

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


    chonkur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("chonkur_deploy"), 'launch','chonkur_control.launch.py')),
        launch_arguments={
            "description_package": "clr_description",
            "description_file": "clr.urdf.xacro",
            "tf_prefix": tf_prefix,
            "use_fake_hardware": use_fake_hardware,
            "headless_mode": headless_mode,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "rviz": rviz,
        }.items(),
    )

    vention_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("vention_rail_deploy"), 'launch','spawn_controllers.launch.py')),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )
    
    ewellix_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ewellix_liftkit_deploy"), 'launch','spawn_controllers.launch.py')),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    clr_controllers_yaml = os.path.join(get_package_share_directory("clr_deploy"), 'config','clr_controllers.yaml')
    lift_rail_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lift_rail_joint_trajectory_controller", 
                   "-c", "controller_manager",
                   "-t", "joint_trajectory_controller/JointTrajectoryController ",
                   "-p", clr_controllers_yaml, 
                   "--inactive"
                  ]
    )
    clr_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["clr_joint_trajectory_controller", 
                   "-c", "controller_manager",
                   "-t", "joint_trajectory_controller/JointTrajectoryController ",
                   "-p", clr_controllers_yaml, 
                   "--inactive"
                  ]
    )

    streaming_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["streaming_controller", 
                   "-c", "controller_manager",
                   "-t", "position_controllers/JointGroupPositionController ",
                   "-p", clr_controllers_yaml, 
                   "--inactive"
                  ]
    )    

    controller_nodes = [chonkur_launch, vention_controllers_launch, ewellix_controllers_launch, lift_rail_controller, clr_controller, streaming_controller]

    return LaunchDescription(declared_arguments + controller_nodes)
