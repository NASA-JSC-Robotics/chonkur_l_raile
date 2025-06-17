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
        self.declare_parameter(
            "target_controller",
            "admittance_joint_trajectory_controller",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING, description="controllers that will always remain active"
            ),
        )
        self.target_controller = self.get_parameter("target_controller").value

        self.get_state_cb_group = ReentrantCallbackGroup()
        self.get_program_state_srv = self.create_client(
            GetProgramState, "/dashboard_client/program_state", callback_group=self.get_state_cb_group
        )

    def initialize(self):
        super().initialize()

        # Wait for the target_controller to be available
        self.wait_for_controller(self.target_controller)

        # Wait for the dashboard client to be available
        self.wait_for_service(self.get_program_state_srv)

        # timer at 0.5 second loop to check controller status and cancel
        self.timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.5, self.timer_callback, callback_group=self.timer_cb_group)
        self.get_logger().info(f"{bcolors.OKBLUE}Chonkur Controller Stopper is running!{bcolors.OKBLUE}")

    def wait_for_controller(self, target_controller, retries=-1):
        attempts = 0
        rate = self.create_rate(1)
        while rclpy.ok():
            self.get_logger().info(f"Waiting for controller: {target_controller}...")
            list_controllers_response = self.call_list_controllers()
            for controller in list_controllers_response.controller:
                if target_controller == controller.name:
                    self.get_logger().info(f"{target_controller} loaded!")
                    return

            attempts += 1
            if retries > 0 and attempts > retries:
                raise RuntimeError(f"Timed out waiting for the controller: {target_controller}")

            self.get_logger().info(f"{bcolors.WARNING}Waiting for controller: {target_controller}...{bcolors.ENDC}")
            rate.sleep()

    def timer_callback(self):
        request = GetProgramState.Request()
        result = self.call_async(self.get_program_state_srv, request)

        if result is None or not result.success:
            self.get_logger().error("was not able to get the state of the program")
            return  # dashboard client publishes its own failure message

        # if we are either paused or stopped ,we treat that as not running
        self.robot_running = result.state.state == ProgramState.PLAYING

        # if we just transitioned to a running state, and the controllers weren't active,
        # start the controllers
        if self.robot_running and not self.controllers_active:
            self.get_logger().info(f"{bcolors.WARNING}Transitioning to running, restarting controllers{bcolors.ENDC}")
            # stop controllers first to get rid of anything that may have happened recently
            self.stop_controllers()
            # start controllers
            self.start_controllers()
        # if robot is either paused or stopped, consistently stop controllers to cancel anything that may have started
        elif not self.robot_running:
            self.get_logger().debug("Robot not running, stopping controllers")
            self.stop_controllers()


def main(args=None):
    rclpy.init(args=args)
    chonkur_controller_stopper = ChonkurControllerStopper()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(chonkur_controller_stopper)

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
