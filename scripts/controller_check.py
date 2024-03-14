#!/usr/bin/env python3
import argparse
import rclpy
from drt_ros2_control_tools.ctrl_stat_tools import ControlStatusClient
from drt_ros2_control_tools.ctrl_config_compiler import bcolors


# TODO: CURRENTLY NOT WORKING, compare_ctrlrs outputs a singular dict now, adapt this script and add error behaviors to compare_ctrlrs!!
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim",
                        default=False,
                        action='store_true',
                        help="Indicate if robot is running in sim")
    args = parser.parse_args()
    rclpy.init()
    print(f'args.sim: {args.sim}')

    ctrl_stat_client = ControlStatusClient(args.sim)
    ctrlr_status = ctrl_stat_client.compare_ctrlrs()
    
    if ctrlr_status:
        print(bcolors.OKGREEN + bcolors.BOLD + 'Listed and spawned:' + bcolors.ENDC)
        for i in range(len(ctrlr_status['spawned']['name'])):
            print('\t - ' + ctrlr_status['spawned']['name'][i] + ' [' + ctrlr_status['spawned']['state'][i] + ']')
        
        print(bcolors.WARNING + bcolors.BOLD + 'Listed and not spawned' + bcolors.ENDC)
        _ = [print('\t -' + name) for name in ctrlr_status['not_spawned']['name']]
    
    ctrl_stat_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
