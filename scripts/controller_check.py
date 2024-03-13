#!/usr/bin/env python3
import rclpy
from drt_ros2_control_tools.ctrl_stat_tools import ControlStatusClient
from drt_ros2_control_tools.ctrl_config_compiler import bcolors


# TODO: CURRENTLY NOT WORKING, compare_ctrlrs outputs a singular dict now, adapt this script and add error behaviors to compare_ctrlrs!!
def main():
    rclpy.init()

    ctrl_stat_client = ControlStatusClient()
    ctrlr_status = ctrl_stat_client.compare_ctrlrs()
    
    print(bcolors.OKGREEN + bcolors.BOLD + 'Listed and spawned:' + bcolors.ENDC)
    for i in range(len(ctrlr_status['spawned']['name'])):
        print('\t - ' + ctrlr_status['spawned']['name'][i] + ' [' + ctrlr_status['spawned']['state'][i] + ']')
    # _ = [print('\t -' + ctrlr['name'] + ' [' + ctrlr['status'] + ']') for ctrlr in ctrlr_status['spawned']]
    
    print(bcolors.WARNING + bcolors.BOLD + 'Listed and not spawned' + bcolors.ENDC)
    _ = [print('\t -' + name) for name in ctrlr_status['not_spawned']['name']]

if __name__ == '__main__':
    main()
