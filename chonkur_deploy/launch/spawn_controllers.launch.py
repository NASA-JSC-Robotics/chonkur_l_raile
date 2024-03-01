import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )    
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # controller_params_file = os.path.join(get_package_share_directory("chonkur_deploy"),'config','chonkur_controllers.yaml')
    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["admittance_controller", 
                   "--controller-manager-timeout","100",
                   "-c", "controller_manager",
                   "-t", "admittance_controller/AdmittanceController",
                #    "-p", controller_params_file,
                   "--inactive"],
    )
    admittance_jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["admittance_joint_trajectory_controller", 
                   "--controller-manager-timeout","100",
                   "-c", "controller_manager",
                   "-t", "joint_trajectory_controller/JointTrajectoryController ",
                #    "-p", controller_params_file,
                   "--inactive"],
    )    

    nodes = [admittance_controller_spawner, 
             admittance_jtc_spawner]    

    return LaunchDescription(declared_arguments + nodes)
