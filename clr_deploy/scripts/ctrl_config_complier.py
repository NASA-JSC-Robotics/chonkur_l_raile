import pathlib
import yaml

configpaths

#PSUDO
# Input file paths
#   configpaths = <all them input filepaths>
configpaths = ["src/chonkur_l_raile/clr_deploy/config/clr_controllers.yaml", 
            "src/chonkur/chonkur_deploy/config/chonkur_controllers.yaml", 
            "src/ewellix_lift_kit/ewellix_liftkit_deploy/config/liftkit_controllers.yaml", 
            "src/vention_rail/vention_rail_deploy/config/rail_controllers.yaml", 
            "src/ros2_robotiq_gripper/robotiq_driver/config/robotiq_hande_controllers.yaml"]

# TODO: def compile_controller_configurations(configuration_paths, output_paths):
# Load config yamls
configs = {}
for config in configpaths:
    configpath = pathlib.Path(config)
    with open(configpath, 'r') as file:
        configs[configpath.stem] = yaml.safe_load(file)

# ID controllers in each filepah
ctrlr_names = []
for config in configs:
    ctrlrs = list(configs[config]['controller_manager']['ros__parameters'].keys())
    if 'update_rate' in ctrlrs:
        ctrlrs.remove('update_rate')
    ctrlr_names += ctrlrs

# ID Duplicate controllers
#   - Specifically, we only need one joint_state_broadcaster
ctrlr_names = list(set(ctrlr_names))
# TODO: Error on duplicated (maybe besides joint_state_broadcaster)

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
with open('compiled_yaml1.yaml', 'w') as file:
    yaml.dump(compiled_cfg, file)
        