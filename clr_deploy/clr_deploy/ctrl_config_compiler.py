import os, sys
import pathlib
import json
import yaml
from typing import List, Dict

import launch.events
from launch.actions import LogInfo
import rclpy

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def compile_controller_configurations(configuration_paths: List[str], output_path: str) -> int:
    """
    # TODO DOC STRING
    """
    # Load config yamls
    configs = {}
    for config in configuration_paths:
        configpath = pathlib.Path(config)
        if not configpath.is_file():
            print(bcolors.HEADER + '(compile_controller_configurations) ' + bcolors.ENDC +
                  bcolors.WARNING + bcolors.UNDERLINE + 'WARNING:' + bcolors.ENDC +
                  bcolors.WARNING + f' failed to find file at {configpath}, skipping...' + bcolors.ENDC)
            continue
        if configpath.suffix != '.yaml':
            print(bcolors.HEADER + '(compile_controller_configurations) ' + bcolors.ENDC +
                  bcolors.WARNING + bcolors.UNDERLINE + 'WARNING:' + bcolors.ENDC +
                  bcolors.WARNING + f' file at {configpath} is not a controller configuration file, skipping...' + bcolors.ENDC)
            continue
        else:
            with open(configpath, 'r') as file:
                configs[configpath.stem] = yaml.safe_load(file)
            if 'controller_manager' not in configs[configpath.stem]:
                print(bcolors.HEADER + '(compile_controller_configurations) ' + bcolors.ENDC +                
                      bcolors.WARNING + bcolors.UNDERLINE + 'WARNING:' + bcolors.ENDC +
                      bcolors.WARNING + f' file at {configpath} is not a controller configuration file, skipping...' + bcolors.ENDC)
                del configs[configpath.stem]
                continue


    # Get all controller names
    ctrlr_names = []
    for config in configs:
        ctrlrs = list(configs[config]['controller_manager']['ros__parameters'].keys())
        if 'update_rate' in ctrlrs:
            ctrlrs.remove('update_rate')
        ctrlr_names += ctrlrs

    # Handle duplicate controllers
    ctrlr_name_count = {name:ctrlr_names.count(name) for name in list(set(ctrlr_names))} # count name occurrences
    for name in ctrlr_name_count:
        if ctrlr_name_count[name] > 1:
            if name != 'joint_state_broadcaster':
                # TODO: ERROR AND EXIT LAUNCH ON DUPLICATE DETECTION BESIDES joint_state_boardcaster
                print(bcolors.HEADER + '(compile_controller_configurations) ' + bcolors.ENDC +
                      bcolors.FAIL + bcolors.BOLD + bcolors.UNDERLINE + 'ERROR:' + bcolors.ENDC +
                      bcolors.FAIL + f' Duplicate controller {name} found, exiting...' + bcolors.ENDC)
                return 1
            else:
                print(bcolors.HEADER + '(compile_controller_configurations) ' + bcolors.ENDC +
                      bcolors.OKCYAN + 'Removing duplicated joint_state_broadcaster controllers' + bcolors.ENDC)
                while ctrlr_name_count[name] > 1:
                    ctrlr_names.remove('joint_state_broadcaster')
                    ctrlr_name_count[name] -= 1


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

    return 0

if __name__ == "__main__":
    cfg_paths = json.loads(sys.argv[1])
    print('cfg_paths: \n')
    print(cfg_paths)
    out_path = sys.argv[2]
    print('out_paths: \n')
    print(out_path)
    compile_controller_configurations(cfg_paths, out_path)