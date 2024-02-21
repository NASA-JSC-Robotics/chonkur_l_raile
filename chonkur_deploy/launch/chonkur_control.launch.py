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
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="chonkur_description"
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="chonkur.urdf.xacro"
        )
    )    

    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    rviz = LaunchConfiguration("rviz")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    controller_params_file = os.path.join(get_package_share_directory("clr_deploy"),'config','all_controllers.yaml')


    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ur_robot_driver"), 'launch','ur_control.launch.py')),
        launch_arguments={
            "ur_type": "ur10e",
            "robot_ip": "192.168.1.102",
            "controller_spawner_timeout": "100",
            "description_package": description_package, # clr_description
            "description_file": description_file, # clr.urdf.xacro
            "tf_prefix": tf_prefix, # 
            "runtime_config_package": "chonkur_deploy", 
            "controllers_file": controller_params_file, # clr_deploy/config/all_controllers.yaml
            "use_fake_hardware": use_fake_hardware, # true
            "headless_mode": headless_mode, # false
            "fake_sensor_commands": fake_sensor_commands, # false
            "initial_joint_controller": initial_joint_controller, # chonkur_joint_trajectory_controller
            "activate_joint_controller": activate_joint_controller, # true
            "launch_rviz": rviz, # false
        }.items(),
    )

    # gripper_controller_yaml = os.path.join(get_package_share_directory("robotiq_driver"), 'config','robotiq_hande_controllers.yaml')
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_hande_controller", 
                   "-c", "controller_manager",
                   "-t", "position_controllers/GripperActionController",
                   "--controller-manager-timeout","100",
                #    "-p", gripper_controller_yaml
                  ]
    )

    gripper_activation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_activation_controller", 
                   "-c", "controller_manager",
                   "-t", "robotiq_controllers/RobotiqActivationController",
                   "--controller-manager-timeout","100",
                #    "-p", gripper_controller_yaml
                  ]
    )

    nodes = [gripper_controller_spawner, 
             gripper_activation_controller_spawner]

    # spawn_controllers_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("chonkur_deploy"), 'launch','spawn_controllers.launch.py')),
    #     launch_arguments={
    #         "use_fake_hardware": use_fake_hardware,
    #     }.items(),
    # )

    # return LaunchDescription(declared_arguments + [base_launch]) #, spawn_controllers_launch] + nodes)
    return LaunchDescription(declared_arguments + [base_launch] + nodes)
