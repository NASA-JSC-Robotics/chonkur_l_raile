#!/usr/bin/env python3
import argparse
import curses
import rclpy
import time

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

    stdscr = curses.initscr()

    rclpy.init()
    ctrl_stat_client = ControlStatusClient(args.sim)
    try:
        while rclpy.ok():
            lines = []
            stdscr.clear()
            ctrlr_status = ctrl_stat_client.compare_ctrlrs()
            if ctrlr_status:
                lines.append('Listed and spawned:')
                # print(bcolors.OKGREEN + bcolors.BOLD + 'Listed and spawned:' + bcolors.ENDC)
                for i in range(len(ctrlr_status['spawned']['name'])):
                    lines.append('\t - ' + ctrlr_status['spawned']['name'][i] + ' [' + ctrlr_status['spawned']['state'][i] + ']')
                    # print('\t - ' + ctrlr_status['spawned']['name'][i] + ' [' + ctrlr_status['spawned']['state'][i] + ']')
                lines.append('Listed and not spawned')
                # print(bcolors.WARNING + bcolors.BOLD + 'Listed and not spawned' + bcolors.ENDC)
                [lines.append('\t -' + name) for name in ctrlr_status['not_spawned']['name']]
                # _ = [print('\t -' + name) for name in ctrlr_status['not_spawned']['name']]
                text = '\n'.join(lines)
                stdscr.addstr(0,0,text)
                stdscr.refresh()
            time.sleep(1.0)
    finally:
        curses.endwin()
        ctrl_stat_client.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
