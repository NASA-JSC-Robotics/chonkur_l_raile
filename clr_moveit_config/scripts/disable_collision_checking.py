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
from rclpy.executors import MultiThreadedExecutor
from moveit2_collision_utils.planning_scene_helper import PlanningSceneHelper
import threading
import sys


def main(args):
    rclpy.init(args=args)
    print(args)

    planning_scene_node = rclpy.create_node("disable_collision_checking")
    planning_scene_helper = PlanningSceneHelper(planning_scene_node)
    executor = MultiThreadedExecutor()
    executor.add_node(planning_scene_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    touch_links_default = ["finger_1_link", "finger_2_link"]
    if len(args) > 1:
        for arg in args[1:]:
            touch_links_default.append(str(arg))

    planning_scene_helper.disable_collision_checking(str(args[0]), touch_links_default)

    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
