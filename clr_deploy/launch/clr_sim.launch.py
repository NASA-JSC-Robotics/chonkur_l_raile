from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os
import yaml

from clr_deploy.ctrl_config_compiler import compile_controller_configurations


def generate_launch_description():

    cfgs = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'sim_controller_configs.yaml')

    with open(cfgs, 'r') as file:
        cfg_paths = yaml.safe_load(file)
    
    cfg_list = [os.path.join(get_package_share_directory(pkg), cfg_paths[pkg]['path']) for pkg in cfg_paths]
    cfg_out = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'compiled_controllers.yaml')

    if not compile_controller_configurations(cfg_list, cfg_out):
        clr_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("clr_deploy"), 'launch','clr_control.launch.py')),
            launch_arguments={
                "use_fake_hardware": "true",
            }.items(),
        )

        return LaunchDescription([clr_launch])
    
    else:
        # TODO: compile_controller_configurations return with error, exit
        return