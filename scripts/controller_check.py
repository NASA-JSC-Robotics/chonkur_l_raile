#!/usr/bin/env python3
import argparse
import time
import rclpy
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
    parser.add_argument("package",
                        type=str,
                        help="Input the package used to run the controllers")
    parser.add_argument("config",
                        type=str,
                        metavar='PATH',
                        help="Filepath to controller configuration YAML file")
    parser.add_argument("--highlight",
                        default=None,
                        type=str,
                        metavar='PATH',
                        help="Filepath to YAML list of controller names to highlight")
    args, unknown = parser.parse_known_args()
    main(args.package, args.config, args.highlight)
