import os, sys
import pathlib
import json
import yaml
from ament_index_python.packages import get_package_share_directory


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

def compile_controller_configurations(config_paths_file: str, output_path: str, debug=False) -> bool:
    """
    Compile ros2_control parameter configuration yamls into a single file for the controller_manager
    
    Positional Arguments:
    config_paths_file -- String filepath to yaml containing packages and paths to control configuration files in the form of 
                         {<pkg>:
                             path: <str path to control config file>} 
    output_path -- Output compiled controller configuation filepath including desired filename
    """

    error_status = False

    if not pathlib.Path(config_paths_file).is_file():
        print(bcolors.HEADER + '(compile_controller_configurations) ' + bcolors.ENDC +
              bcolors.FAIL + bcolors.BOLD + bcolors.UNDERLINE + 'ERROR:' + bcolors.ENDC +
              bcolors.FAIL + f' file at {config_paths_file} not found, exiting...' + bcolors.ENDC)
        error_status = True
        return error_status

    with open(config_paths_file, 'r') as file:
        cfgs = yaml.safe_load(file)
    
    configuration_paths = [os.path.join(get_package_share_directory(pkg), cfgs[pkg]['path']) for pkg in cfgs]

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
                print(bcolors.HEADER + '(compile_controller_configurations) ' + bcolors.ENDC +
                      bcolors.FAIL + bcolors.BOLD + bcolors.UNDERLINE + 'ERROR:' + bcolors.ENDC +
                      bcolors.FAIL + f' Duplicate controller {name} found, exiting...' + bcolors.ENDC)
                error_status = True
                return error_status
            else:
                if debug:
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

    return error_status

if __name__ == "__main__":
    cfg_paths = json.loads(sys.argv[1])
    print('cfg_paths: \n')
    print(cfg_paths)
    out_path = sys.argv[2]
    print('out_paths: \n')
    print(out_path)
    compile_controller_configurations(cfg_paths, out_path)
