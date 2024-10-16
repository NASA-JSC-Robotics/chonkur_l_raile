from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
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
            "controllers_file",
            default_value="chonkur_controllers.yaml",
            description="Select the controller configuration yaml file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="chonkur_deploy",
            description="Defines the package that contains the controllers_file",
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
            "enable_admittance",
            default_value="false",
            description="Allow the admittance controllers to spawn",
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
    controllers_file = LaunchConfiguration("controllers_file")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    enable_admittance = LaunchConfiguration("enable_admittance")
    rviz = LaunchConfiguration("rviz")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    launches = []

    # NOTE: There is an issue with the UR scripting interface that prevents it from clean shut downs
    # when it is not connected to an actual robot. For instance, when running the simulated interface.
    # This means the urscript_interface node will hang, and ultimately require a sigkill in order to
    # terminate the process. It's annoying, but in future versions of the driver that node is not
    # launched when using mock hardware.
    #
    # https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/838
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ur_robot_driver"), 'launch','ur_control.launch.py')),
        launch_arguments={
            "ur_type": "ur10e",
            "robot_ip": "192.168.1.102",
            "controller_spawner_timeout": "100",
            "description_package": description_package,
            "description_file": description_file,
            "tf_prefix": tf_prefix,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "use_fake_hardware": use_fake_hardware,
            "headless_mode": headless_mode,
            "fake_sensor_commands": fake_sensor_commands,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "launch_rviz": rviz,
        }.items(),
    )
    launches.append(base_launch)

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_hande_controller",
                   "-c", "controller_manager",
                   "-t", "position_controllers/GripperActionController",
                   "--controller-manager-timeout","100",
                  ]
    )

    gripper_activation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_activation_controller",
                   "-c", "controller_manager",
                   "-t", "robotiq_controllers/RobotiqActivationController",
                   "--controller-manager-timeout","100",
                  ]
    )

    nodes = [gripper_controller_spawner,
             gripper_activation_controller_spawner]

    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("chonkur_deploy"), 'launch', 'spawn_controllers.launch.py')),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
        }.items(),
        condition=IfCondition(enable_admittance)
    )
    launches.append(spawn_controllers_launch)


    return LaunchDescription(declared_arguments + launches + nodes)
