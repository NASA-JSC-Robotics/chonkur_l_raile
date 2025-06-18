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
import unittest

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from clr_safety.controller_stopper_base import ControllerStopperBase


class ControllerStopperTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def test_construction(self):
        stopper = ControllerStopperBase()

        self.assertFalse(stopper.manage_servo)
        self.assertEqual(stopper.consistent_controllers, [])
        self.assertEqual(stopper.stopped_controllers, [])
        self.assertTrue(stopper.controllers_active)

    def test_stop_start_controllers(self):
        stopper = ControllerStopperBase()

        # Construct a consistent controller and manually add it to the stopper
        consistent_controller = ControllerState()
        consistent_controller.name = "consistent_controller"
        consistent_controller.state = "active"
        stopper.consistent_controllers.append(consistent_controller.name)

        test_controller = ControllerState()
        test_controller.name = "test_controller"
        test_controller.state = "active"

        # Configure expectations for initial call
        # Given two controllers, one consistent one not
        resp = ListControllers.Response()
        resp.controller = [consistent_controller, test_controller]

        # When we call to stop the controllers, only the non-consistent controller should stop
        controllers_to_stop_expected = ["test_controller"]
        controllers_to_start_expected = []

        def call_list_controllers_stub():
            return resp

        def call_switch_controllers_stub(controllers_to_stop=[], controllers_to_start=[]):
            self.assertEqual(controllers_to_stop_expected, controllers_to_stop)
            self.assertEqual(controllers_to_start_expected, controllers_to_start)
            return True

        def call_pause_servo_stub(active):
            # Should not happen
            self.fail("We should not call pause_servo")

        # Install stubs
        stopper.call_list_controllers = call_list_controllers_stub
        stopper.call_switch_controllers = call_switch_controllers_stub
        stopper.call_pause_servo = call_pause_servo_stub

        # Verify call
        stopper.stop_controllers()

        # When we call to start controllers, only the non-consisten controller should be started
        controllers_to_stop_expected = []
        controllers_to_start_expected = ["test_controller"]

        # Verify call
        stopper.start_controllers()

    def test_start_stop_servo(self):
        # Create a stub servo node and let rclpy find it
        stopper = ControllerStopperBase(servo_node_name="servo_node_test")
        self.assertTrue(stopper.manage_servo)

        # When calling to stop controllers while managing servo, we expect a pause servo call
        expected_servo_paused = True
        self.servo_called = False

        def call_list_controllers_stub():
            return ListControllers.Response()

        def call_switch_controllers_stub(controllers_to_stop=[], controllers_to_start=[]):
            return True

        def call_pause_servo_stub(active):
            self.servo_called = True
            self.assertEqual(active, expected_servo_paused)

        def servo_node_exists_stub():
            return True

        # Install stubs
        stopper.call_list_controllers = call_list_controllers_stub
        stopper.call_switch_controllers = call_switch_controllers_stub
        stopper.call_pause_servo = call_pause_servo_stub
        stopper.servo_node_exists = servo_node_exists_stub

        # Execute the stop call, and ensure the stub was triggered
        stopper.stop_controllers()
        self.assertTrue(self.servo_called)

        # When starting controllers, we expect servo to be unpaused
        expected_servo_paused = False
        self.servo_called = False

        # Re-execute
        stopper.start_controllers()
        self.assertTrue(self.servo_called)
