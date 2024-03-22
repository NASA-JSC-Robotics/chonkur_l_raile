#!/usr/bin/env python3
import argparse
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from drt_ros2_control_tools.ctrl_stat_tools import ControlStatusClient


def main(pkg, sim, highlight):
    rclpy.init()
    clr_stat_client = ControlStatusClient(pkg, sim, highlight)
    
    executor = MultiThreadedExecutor()
    executor.add_node(clr_stat_client)
    try:
        executor.spin()
    except KeyboardInterrupt:
        clr_stat_client.exit()
        clr_stat_client.get_logger().info('Keyboard interrupt, shutting down...')
    
    # rclpy.spin(clr_stat_client)

    # while rclpy.ok():
    #     clr_stat_client.show_compare()
    #     time.sleep(1.0)
    # clr_stat_client.exit()
    
    clr_stat_client.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("package",
                        help="Input the package used to run the controllers")
    parser.add_argument("-s", "--sim",
                        default=False,
                        action='store_true',
                        help="Flag to indicate robot is running in sim")
    parser.add_argument("--highlight",
                        default=None,
                        type=str,
                        metavar='PATH',
                        help="Filepath to YAML list of controller names to highlight")
    args, unknown = parser.parse_known_args()
    main(args.sim)
