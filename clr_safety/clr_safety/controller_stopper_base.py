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

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from controller_manager_msgs.srv import ListControllers, SwitchController
from std_srvs.srv import SetBool


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


class ControllerStopperBase(Node):
    """This class cancels the stopping and starting controllers based on some event (implemented by a derived class).

    Args:
        Node (Node): inherits the Node class
    """

    def __init__(
        self, node_name="controller_stopper", controller_manager_name="/controller_manager", servo_node_name=""
    ):
        """the constructor for the ControllerStopperBase Class, which creates can handle stopping and starting
        controllers.

        Args:
            node_name (str, optional): the name of the node that will be created. Defaults to 'controller_stopper'
            controller_manager_name (str, optional): the name of the controller manager node (with namespace) to
            use for service call prefixes. defaults to '/controller_manager' servo_node_name (str, optional): the
            name of servo node (leave blank if there is none). defaults to ''
        """
        # initialize parent node name as
        super().__init__(node_name)

        # use the same callback group for controller manager services because we don't want to be listing controllers
        # as we are switching them
        self.cm_cb_group = MutuallyExclusiveCallbackGroup()

        # setup services for switch and list controllers
        self.switch_controllers_srv = self.create_client(
            SwitchController, controller_manager_name + "/switch_controller", callback_group=self.cm_cb_group
        )
        self.list_controllers_srv = self.create_client(
            ListControllers, controller_manager_name + "/list_controllers", callback_group=self.cm_cb_group
        )

        # if the user doesn't provide the name of the servo node, assume we don't care
        self.servo_node_name = servo_node_name
        self.manage_servo = self.servo_node_name != ""
        self.servo_node_found = False

        # only try to hook up to the node if we are managing servo as well
        if self.manage_servo:
            # use a different callback group for servo pausing/unpausing so that can run in parallel
            self.servo_cb_group = MutuallyExclusiveCallbackGroup()
            self.pause_servo_srv = self.create_client(
                SetBool, self.servo_node_name + "/pause_servo", callback_group=self.servo_cb_group
            )

        # get parameter of list of strings for consistent_controllers (default to nothing)
        self.declare_parameter(
            "consistent_controllers",
            [""],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY, description="controllers that will always remain active"
            ),
        )
        self.consistent_controllers = self.get_parameter("consistent_controllers").value
        if self.consistent_controllers == [""]:
            self.consistent_controllers = []

        # variable to keep track of which controllers have been stopped so they can be restarted
        self.stopped_controllers = []

        # variable to keep track of whether we are in a state where we have restarted controllers
        self.controllers_active = True

    def initialize(self):
        """Waits for the relevant states to be ready before starting."""

        # wait until all required services are available from controller_manager
        self.get_logger().info(
            "Waiting for switch controller service to come up on controller_manager/switch_controller"
        )
        self.wait_for_service(self.switch_controllers_srv)
        self.wait_for_service(self.list_controllers_srv)

    def wait_for_service(self, client, timeout=1, retries=-1):
        """Waits for the ROS 2 service to be available at the specified timeout.

        If retries < 0 this will wait forever.
        """
        attempts = 0
        while rclpy.ok():
            if client.wait_for_service(timeout):
                self.get_logger().info(f"{client.srv_name} found!")
                return
            attempts += 1
            if retries > 0 and attempts > retries:
                raise RuntimeError(f"Timed out waiting for service to be available: {client.srv_name}")
            self.get_logger().info(
                f"{bcolors.WARNING}Waiting for service {client.srv_name} to be available...{bcolors.ENDC}"
            )

    def call_async(self, client, request, timeout=3):
        """Calls the provided client and waits the timeout for a response.

        Returns the response if available, or else None.
        """
        future = client.call_async(request)

        start_time = self.get_clock().now()
        while not future.done():
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed_time > timeout:
                self.get_logger().warn(f"Service call to {client.srv_name} timed out after {timeout}s")
                future.cancel()
                return None

        return future.result()

    def servo_node_exists(self):
        """Returns true if the servo node was found in active ROS session. And logs a message if the state
        changes from one to another.
        """
        # list available nodes and look for the servo node in it
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        for node, _ in node_names_and_namespaces:
            if node == self.servo_node_name:
                # if we found the node, but we didn't have it previously, let the user know we are now tracking it
                if not self.servo_node_found:
                    self.get_logger().info(f'Servo node "{self.servo_node_name}" is now being handled.')
                self.servo_node_found = True
                # exit early
                return True

        # if last cycle we found the node, but now we can't find it, let the user know that we are no longer tracking
        name = self.servo_node_name  # shorter name to not break pre-commit
        if self.servo_node_found:
            self.get_logger().info(
                f'{bcolors.WARNING}Was asked to monitor the servo node "{name}" but it was not found{bcolors.ENDC}'
            )
        self.servo_node_found = False

        return False

    def call_list_controllers(self):
        list_controllers_request = ListControllers.Request()
        list_controllers_response = self.call_async(self.list_controllers_srv, list_controllers_request)
        return list_controllers_response

    def call_switch_controllers(self, controllers_to_stop=[], controllers_to_start=[]):
        switch_controller_request = SwitchController.Request()
        # set strictness to strict to show that this fails if anything went wrong
        switch_controller_request.strictness = SwitchController.Request.STRICT
        switch_controller_request.timeout.nanosec = 500000000  # 0.5s
        switch_controller_request.deactivate_controllers = controllers_to_stop
        switch_controller_request.activate_controllers = controllers_to_start

        # Call the service
        switch_controllers_response = self.call_async(self.switch_controllers_srv, switch_controller_request)

        if switch_controllers_response:
            return switch_controllers_response.ok
        else:
            return False

    def call_pause_servo(self, servo_active):
        pause_servo_request = SetBool.Request()
        pause_servo_request.data = servo_active
        pause_servo_response = self.call_async(self.pause_servo_srv, pause_servo_request)

        if pause_servo_response is None or not pause_servo_response.success:
            self.get_logger().error("Could not pause servo node")

    def stop_controllers(self):
        """Deactivates all of the controllers that are not listed as consistent.
        This can be called several times without causing issues. This is useful because
        some controllers may be turned back on in some cases, like if a trajectory is accidentally
        performed while the robot is disabled
        """
        # only save the controllers to restart if this is the first cycle when the controllers were active
        # this makes it so we don't get conflicts if we tried to load another controller that needed the same
        # interfaces as one that was already active
        save_controllers = self.controllers_active

        # each new time this is called, we will repopulate which controllers we are stopping
        controllers_to_stop = []

        # Get list of controllers from the CM
        list_controllers_response = self.call_list_controllers()

        # add controllers that are active and not consistent to the list to stop
        for controller in list_controllers_response.controller:
            if controller.state == "active":
                if controller.name not in self.consistent_controllers:
                    controllers_to_stop.append(controller.name)

        # if there are any to stop, call switch_controllers to stop them
        if controllers_to_stop:
            if not self.call_switch_controllers(controllers_to_stop=controllers_to_stop):
                self.get_logger().error("Could not deactivate requested controllers")
            else:
                self.controllers_active = False

        # save the stopped controllers only if we are in a state where the controllers were active
        if save_controllers:
            self.stopped_controllers = controllers_to_stop

        # if we are managing the servo controller, also pause the servo node
        if self.manage_servo:
            if self.servo_node_exists():
                self.call_pause_servo(True)

    def start_controllers(self):
        """Reactivates all of the controllers that have been stopped by this node."""

        # only run this if we have logged that some controllers have been stopped
        if self.stopped_controllers:
            if not self.call_switch_controllers(controllers_to_start=self.stopped_controllers):
                self.get_logger().error("Could not activate requested controllers")
            else:
                self.controllers_active = True

        # if we are managing the servo controller, restart the servo node
        if self.manage_servo:
            if self.servo_node_exists():
                self.call_pause_servo(False)

        # clear stopped controllers for the next round to populate
        self.stopped_controllers.clear()
