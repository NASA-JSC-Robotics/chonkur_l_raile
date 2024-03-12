#!/usr/bin/env python3
import rclpy
from drt_ros2_control_tools.ctrl_stat_tools import ControlStatusClient
from drt_ros2_control_tools.ctrl_config_compiler import bcolors


# TODO: put main in a different file
def main():
    rclpy.init()

    ctrl_stat_client = ControlStatusClient()
    spawned, not_spawned = ctrl_stat_client.compare_ctrlrs()
    
    print(bcolors.OKGREEN + bcolors.BOLD + 'Listed and spawned:' + bcolors.ENDC)
    _ = [print('\t -' + name) for name in spawned]
    
    print(bcolors.WARNING + bcolors.BOLD + 'Listed and not spawned' + bcolors.ENDC)
    _ = [print('\t -' + name) for name in not_spawned]

if __name__ == '__main__':
    main()
