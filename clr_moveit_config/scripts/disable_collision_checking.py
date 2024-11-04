#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from moveit2_collision_utils.planning_scene_helper import PlanningSceneHelper
from moveit2_collision_utils.collision_object import CollisionObjectCreator
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
import threading
import time
from copy import deepcopy
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
