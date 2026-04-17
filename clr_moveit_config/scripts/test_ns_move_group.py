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

import argparse
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from rclpy.action import ActionClient

# This is a simple test script for executing a hard-coded plan and execute for namespaced CLR.
# Exists just to confirm that it works.


def main():
    parser = argparse.ArgumentParser(description="Test MoveIt planning and execution")
    parser.add_argument("--namespace", default="", help="MoveGroup namespace (e.g. 'left')")
    parser.add_argument("--tf-prefix", default="", help="Joint name prefix (e.g. 'left_')")
    parser.add_argument("--execute", action="store_true", help="Execute the planned trajectory")
    args = parser.parse_args()

    rclpy.init()
    node = Node("test_moveit", namespace=args.namespace)

    client = ActionClient(node, MoveGroup, "move_action")
    node.get_logger().info("Waiting for move_action server...")
    client.wait_for_server()
    node.get_logger().info("Connected!")

    p = args.tf_prefix
    joint_names = [
        f"{p}shoulder_pan_joint",
        f"{p}shoulder_lift_joint",
        f"{p}elbow_joint",
        f"{p}wrist_1_joint",
        f"{p}wrist_2_joint",
        f"{p}wrist_3_joint",
    ]

    # Just hardcoding these for now...
    joint_values = [0.5, -1.5707, 0.0, 0.0, 0.0, 0.0]

    goal = MoveGroup.Goal()
    goal.request = MotionPlanRequest()
    goal.request.group_name = "ur_manipulator"
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 5.0

    constraints = Constraints()
    for name, value in zip(joint_names, joint_values):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = value
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)

    goal.request.goal_constraints.append(constraints)
    goal.planning_options.plan_only = not args.execute

    action = "plan+execute" if args.execute else "plan-only"
    node.get_logger().info(f"Sending {action} request...")
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error("Goal rejected!")
        rclpy.shutdown()
        return

    node.get_logger().info("Goal accepted, waiting for result...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result().result
    if result.error_code.val == 1:
        points = len(result.planned_trajectory.joint_trajectory.points)
        node.get_logger().info(f"Success! Trajectory has {points} points.")
    else:
        node.get_logger().error(f"Failed with error code: {result.error_code.val}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
