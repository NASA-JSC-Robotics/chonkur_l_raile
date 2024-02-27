import os, sys
import pathlib
import json
import yaml
from typing import List, Dict

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

if __name__ == "__main__":
    cfg_paths = json.loads(sys.argv[1])
    print('cfg_paths: \n')
    print(cfg_paths)
    out_path = sys.argv[2]
    print('out_paths: \n')
    print(out_path)
    compile_controller_configurations(cfg_paths, out_path)