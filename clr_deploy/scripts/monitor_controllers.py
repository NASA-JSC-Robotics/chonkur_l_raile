#!/usr/bin/env python3
import argparse
import os
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from drt_ros2_control_tools.ctrl_stat_tools import ControlStatusClientCurses


def main(pkg, sim, highlight):
    rclpy.init()
    clr_stat_client = ControlStatusClientCurses(pkg, sim, highlight)
    while rclpy.ok():
        try:
            clr_stat_client.show_compare()
            time.sleep(1)
        except KeyboardInterrupt:
            clr_stat_client.exit()
            clr_stat_client.get_logger().info('Keyboard interrupt, shutting down...')
            clr_stat_client.destroy_node()
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--sim",
                        default=False,
                        action='store_true',
                        help="Select if running robot in sim")
    args, unknown = parser.parse_known_args()
    if args.sim:
        config = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'sim_controllers.yaml')
    else:
        config = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'hardware_controllers.yaml')

    highlight = os.path.join(get_package_share_directory('clr_deploy'), 'config', 'highlighted_controllers.yaml')
    main('clr_deploy', config, highlight)

