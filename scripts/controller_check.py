#!/usr/bin/env python3
import argparse
import curses
import rclpy
import time
import yaml

from drt_ros2_control_tools.ctrl_stat_tools import ControlStatusClient
from drt_ros2_control_tools.ctrl_config_compiler import bcolors

def main():
 
    # argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--sim",
                        default=False,
                        action='store_true',
                        help="Indicate if robot is running in sim")
    parser.add_argument("--highlight",
                        type=str,
                        metavar='PATH',
                        help="Filepath to YAML list of controller names to highlight")
    args, unknown = parser.parse_known_args()

    # curses
    stdscr = curses.initscr()
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_WHITE)
    curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_RED)

    # yaml
    if args.highlight:
        with open(args.highlight, 'r') as file:
            highlighted = yaml.safe_load(file)

    # compare controllers and display loop (1 Hz)
    rclpy.init()
    ctrl_stat_client = ControlStatusClient(args.sim)
    try:
        while rclpy.ok():
            stdscr.clear()
            line = 0
            ctrlr_status = ctrl_stat_client.compare_ctrlrs()
            if ctrlr_status:
                stdscr.addstr(line, 0, 'Listed and spawned:')
                line += 1
                for i in range(len(ctrlr_status['spawned']['name'])):
                    name = ctrlr_status['spawned']['name'][i]
                    state = ctrlr_status['spawned']['state'][i]
                    if args.highlight and name in highlighted:
                        color = 2
                    else:
                        color = 1
                    stdscr.addstr(line, 0,
                                  '\t - ' + name + ' [' + state + ']',
                                  curses.color_pair(color))
                    line += 1
                stdscr.addstr(line, 0, 'Listed and not spawned')
                line += 1
                for i in range(len(ctrlr_status['not_spawned']['name'])):
                    name = ctrlr_status['not_spawned']['name'][i]
                    if args.highlight and name in highlighted:
                        color = 4
                    else:
                        color = 3
                    stdscr.addstr(line, 0,
                                  '\t - ' + name,
                                  curses.color_pair(color))
                    line += 1
                stdscr.refresh()
            time.sleep(1.0)
    finally:
        curses.endwin()
        ctrl_stat_client.destroy_node()

if __name__ == '__main__':
    main()
