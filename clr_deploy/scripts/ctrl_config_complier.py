import os
import pathlib
import yaml


from typing import List, Dict


configpaths =  ["src/chonkur_l_raile/clr_deploy/config/clr_controllers.yaml", 
                "src/chonkur/chonkur_deploy/config/chonkur_controllers.yaml", 
                "src/ewellix_lift_kit/ewellix_liftkit_deploy/config/liftkit_controllers.yaml", 
                "src/vention_rail/vention_rail_deploy/config/rail_controllers.yaml", 
                "src/ros2_robotiq_gripper/robotiq_driver/config/robotiq_hande_controllers.yaml"]

def compile_controller_configurations(configuration_paths: List[str], output_path: str) -> Dict:
    """
    # TODO DOC STRING
    """
    # Load config yamls
    configs = {}
    for config in configuration_paths:
        #TODO: Error if path doesn't exist or not a configuration yaml
        configpath = pathlib.Path(config)
        with open(configpath, 'r') as file:
            configs[configpath.stem] = yaml.safe_load(file)

    # Get all controller names
    ctrlr_names = []
    for config in configs:
        ctrlrs = list(configs[config]['controller_manager']['ros__parameters'].keys())
        if 'update_rate' in ctrlrs:
            ctrlrs.remove('update_rate')
        ctrlr_names += ctrlrs

    # Handle duplicate controllers
    # TODO: Remove duplicate joint_state_broadcasters
    # TODO: Error and exit on any other duplicated controller name
    ctrlr_names = list(set(ctrlr_names))

    # Compile controller configs
    compiled_cfg = {}
    compiled_cfg['controller_manager'] = {}
    compiled_cfg['controller_manager']['ros__parameters'] = {}
    for config in configs:
        for ctrlr_name in ctrlr_names:
            if ctrlr_name in configs[config]:
                compiled_cfg[ctrlr_name] = configs[config][ctrlr_name] # Compile controller parameters
            if ctrlr_name in configs[config]['controller_manager']['ros__parameters']:
                compiled_cfg['controller_manager']['ros__parameters'][ctrlr_name] = configs[config]['controller_manager']['ros__parameters'][ctrlr_name] # Compile controller types

    # Export yaml
    with open(output_path, 'w') as file:
        yaml.dump(compiled_cfg, file)

    return
