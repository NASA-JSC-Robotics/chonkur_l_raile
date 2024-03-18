from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os
import yaml

from clr_deploy.ctrl_config_compiler import compile_controller_configurations


def generate_launch_description():
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_admittance",
            default_value="false",
            description="Allow the admittance controllers to spawn",
        )
    )

    enable_admittance = LaunchConfiguration("enable_admittance")

    cfgs = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'hardware_controller_configs.yaml')
    cfg_out = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'hardware_controllers.yaml')

    compile_error = compile_controller_configurations(cfgs, cfg_out)

    if not compile_error:
        clr_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("clr_deploy"), 'launch','clr_control.launch.py')),
            launch_arguments={
                "controllers_file": cfg_out,
                "enable_admittance": enable_admittance,
            }.items(),
        )        
        return LaunchDescription(declared_arguments + [clr_launch])
    else:
        Shutdown(
            reason='Control configuration compiled failed, exiting clr_sim.launch.py'
        )
        return