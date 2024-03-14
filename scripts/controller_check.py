#!/usr/bin/env python3
import argparse
import curses
import rclpy
import time

from drt_ros2_control_tools.ctrl_stat_tools import ControlStatusClient
from drt_ros2_control_tools.ctrl_config_compiler import bcolors

# color lookup table for curses
ctrlr_colors = {
    'spawned':{
        'admittance_controller': 1,
        'admittance_joint_trajectory_controller': 1,
        'clr_joint_trajectory_controller': 2,
        'faked_forces_controller': 1,
        'force_torque_sensor_broadcaster': 1,
        'forward_position_controller': 1,
        'forward_velocity_controller': 1,
        'io_and_status_controller': 1,
        'joint_state_broadcaster': 1,
        'joint_trajectory_controller': 2,
        'lift_position_trajectory_controller': 2,
        'lift_rail_joint_trajectory_controller': 2,
        'rail_estop_controller': 1,
        'rail_position_trajectory_controller': 2,
        'robotiq_activation_controller': 2,
        'robotiq_gripper_hande_controller': 2,
        'scaled_joint_trajectory_controller': 1,
        'speed_scaling_state_broadcaster': 1,
        'streaming_controller': 1,
        },
    'not_spawned':{
        'admittance_controller': 3,
        'admittance_joint_trajectory_controller': 3,
        'clr_joint_trajectory_controller': 4,
        'faked_forces_controller': 3,
        'force_torque_sensor_broadcaster': 3,
        'forward_position_controller': 3,
        'forward_velocity_controller': 3,
        'io_and_status_controller': 3,
        'joint_state_broadcaster': 3,
        'joint_trajectory_controller': 4,
        'lift_position_trajectory_controller': 4,
        'lift_rail_joint_trajectory_controller': 4,
        'rail_estop_controller': 3,
        'rail_position_trajectory_controller': 4,
        'robotiq_activation_controller': 4,
        'robotiq_gripper_hande_controller': 4,
        'scaled_joint_trajectory_controller': 3,
        'speed_scaling_state_broadcaster': 3,
        'streaming_controller': 3,
        },
}

def main():

    # argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim",
                        default=False,
                        action='store_true',
                        help="Indicate if robot is running in sim")
    args = parser.parse_args()

    # curses
    stdscr = curses.initscr()
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_WHITE)
    curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_RED)

    # compare controllers and display loop (1 Hz)
    rclpy.init()
    ctrl_stat_client = ControlStatusClient(args.sim)

    try:
        while rclpy.ok():
            # lines = []
            stdscr.clear()
            line = 0
            ctrlr_status = ctrl_stat_client.compare_ctrlrs()
            if ctrlr_status:
                stdscr.addstr(line, 0, 'Listed and spawned:')
                line += 1
                for i in range(len(ctrlr_status['spawned']['name'])):
                    stdscr.addstr(line, 0,
                                  '\t - ' + ctrlr_status['spawned']['name'][i] + ' [' + ctrlr_status['spawned']['state'][i] + ']',
                                  curses.color_pair(ctrlr_colors['spawned'][ctrlr_status['spawned']['name'][i]]))
                    line += 1
                stdscr.addstr(line, 0, 'Listed and not spawned')
                line += 1
                for i in range(len(ctrlr_status['not_spawned']['name'])):
                    stdscr.addstr(line, 0,
                                  '\t - ' + ctrlr_status['not_spawned']['name'][i],
                                  curses.color_pair(ctrlr_colors['not_spawned'][ctrlr_status['not_spawned']['name'][i]]))
                    line += 1
                stdscr.refresh()
            time.sleep(1.0)

    finally:
        curses.endwin()
        ctrl_stat_client.destroy_node()

if __name__ == '__main__':
    main()
