import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    clr_ctrl_cfg_path = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'clr_controllers.yaml')
    chonkur_ctrl_cfg_path = os.path.join(get_package_share_directory('chonkur_deploy'), 'config', 'chonkur_controllers.yaml')
    liftkit_ctrl_cfg_path = os.path.join(get_package_share_directory('ewellix_liftkit_deploy'), 'config', 'liftkit_controllers.yaml')
    rail_ctrl_cfg_path = os.path.join(get_package_share_directory('vention_rail_deploy'), 'config', 'rail_controllers.yaml')
    hande_ctrl_cfg_path = os.path.join(get_package_share_directory('robotiq_driver'), 'config', 'robotiq_hande_controllers.yaml')
    ctrl_cfg_paths = [clr_ctrl_cfg_path, chonkur_ctrl_cfg_path, liftkit_ctrl_cfg_path, rail_ctrl_cfg_path, hande_ctrl_cfg_path]
    
    compiled_ctrl_cfg_path = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'compiled_controllers.yaml')
    

    compiler_path = os.path.join(get_package_share_directory('clr_deploy'), 'ctrl_config_complier.py')
    compiler_process = ExecuteProcess(
        cmd=["python3", compiler_path, json.dumps(ctrl_cfg_paths), compiled_ctrl_cfg_path],
        output="screen",
    )

    return LaunchDescription([compiler_process])