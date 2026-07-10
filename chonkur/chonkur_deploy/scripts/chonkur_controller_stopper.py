#!/usr/bin/env python3
#
# Copyright (c) 2025, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import rclpy
import sys
import threading

from ur_dashboard_msgs.srv import GetProgramState
from ur_dashboard_msgs.msg import ProgramState
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from clr_safety.controller_stopper_base import ControllerStopperBase, bcolors


class ChonkurControllerStopper(ControllerStopperBase):
    """This class cancels the stopping and starting controllers based on some event (implemented by a derived class).

    Args:
        ControllerStopperBase (ControllerStopperBase): inherits the ControllerStopperBase class
    """

    def __init__(self):
        """the constructor for the ChonkurControllerStopper Class, which creates can handle stopping/starting
        controllers, and pausing/unpausing servo when the program stops running.
        """

        # create instance of ControllerStopperBase with node name, and telling it we do want it to manage the servo node
        super().__init__(node_name="chonkur_controller_stopper", servo_node_name="servo_server")

        # Given the relative time it takes for chonkur to come up, we wait for a specific controller to be loaded
        # to be reasonably confident that the relevant controllers are loaded into the CM.
        self.node.declare_parameter(
            "target_controller",
            "admittance_joint_trajectory_controller",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING, description="controllers that will always remain active"
            ),
        )
        self.target_controller = self.node.get_parameter("target_controller").value

        self.get_state_cb_group = ReentrantCallbackGroup()
        self.get_program_state_srv = self.node.create_client(
            GetProgramState, "/dashboard_client/program_state", callback_group=self.get_state_cb_group
        )

    def initialize(self):
        super().initialize()

        # Wait until the controller manager has finished spawning all controllers
        self.wait_for_controllers_done_loading()

        # Wait for the dashboard client to be available
        self.wait_for_service(self.get_program_state_srv)

        # timer at 0.5 second loop to check controller status and cancel
        self.timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.node.create_timer(0.5, self.timer_callback, callback_group=self.timer_cb_group)
        self.node.get_logger().info(f"{bcolors.OKBLUE}Chonkur Controller Stopper is running!{bcolors.OKBLUE}")

    def wait_for_controllers_done_loading(self, timeout_s=2.0):
        """
        Wait until a new controller has not been seen for the last <timeout_s> seconds. This is used as a proxy for
        understanding when the controller manager has loaded. Setting this value too low will mean that you might start
        the controller stopper too early, and therefore accidentally disable any controllers that come up afterwards.
        Setting this value too high will mean that the you have to wait a while after the controllers are done loading
        before the controller stopper starts, therefore potentially causing unsafe situations if you

        Args:
            timeout_s (float, optional): Time in seconds since last controller was loaded to be considered done loading
            controllers. Defaults to 2.0.
        """
        check_controllers_rate = self.node.create_rate(2)  # check every 0.5 s
        seen_controllers = []  # keep track of the controllers we have seen so far on bringup
        last_new_controller_time = None
        timeout_duration = rclpy.duration.Duration(seconds=timeout_s)

        while rclpy.ok():
            # get current controllers loaded, and see if there is anything new
            list_controllers_response = self.call_list_controllers()
            current_controllers = [c.name for c in list_controllers_response.controller]
            new_controllers_loaded = current_controllers != seen_controllers

            # if there are new controllers, record the time
            if new_controllers_loaded:
                seen_controllers = current_controllers
                last_new_controller_time = self.node.get_clock().now()

            # if we have waited longer than <timeout_s>, stop blocking
            if last_new_controller_time is not None:
                elapsed = self.node.get_clock().now() - last_new_controller_time
                if elapsed >= timeout_duration:
                    self.node.get_logger().info(
                        f"No new controllers for {elapsed.nanoseconds / 1e9:.1f}s. Controller manager done loading."
                    )
                    return

            check_controllers_rate.sleep()

    def timer_callback(self):
        request = GetProgramState.Request()
        result = self.call_async(self.get_program_state_srv, request)

        if result is None or not result.success:
            self.node.get_logger().error("was not able to get the state of the program")
            return  # dashboard client publishes its own failure message

        # if we are either paused or stopped ,we treat that as not running
        self.robot_running = result.state.state == ProgramState.PLAYING

        # if we just transitioned to a running state, and the controllers weren't active,
        # start the controllers
        if self.robot_running and not self.controllers_active:
            self.node.get_logger().info(
                f"{bcolors.WARNING}Transitioning to running, restarting controllers{bcolors.ENDC}"
            )
            # stop controllers first to get rid of anything that may have happened recently
            self.stop_controllers()
            # start controllers
            self.start_controllers()
        # if robot is either paused or stopped, consistently stop controllers to cancel anything that may have started
        elif not self.robot_running:
            self.node.get_logger().debug("Robot not running, stopping controllers")
            self.stop_controllers()


def main(args=None):
    rclpy.init(args=args)
    chonkur_controller_stopper = ChonkurControllerStopper()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(chonkur_controller_stopper.node)

    # Start the initialization process in the background, this should terminate on its own if all services
    # and controllers are available
    initalize_thread = threading.Thread(target=chonkur_controller_stopper.initialize, daemon=True)
    initalize_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
