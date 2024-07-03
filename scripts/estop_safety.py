#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from drt_ros2_control_tools.estop_safety_class import EStopSafety
from rclpy.executors import MultiThreadedExecutor 

def main(args=None):

    ur_estop_values = [3,5,6,7,11,12,13]
    rclpy.init(args=args)

    estop_t = '/io_and_status_controller/safety_mode'
    list_controllers_service = '/controller_manager/list_controllers'
    #print("mina sanity check")

    controller_types_list = ["joint_trajectory_controller/JointTrajectoryController", "position_controllers/GripperActionController"]

    cancel_list =  ["/follow_joint_trajectory/_action/cancel_goal", "/gripper_cmd/_action/cancel_goal"]

    try:
        estop_node = EStopSafety(estop_topic = estop_t, stop_values = ur_estop_values, list_controllers_service_name = '/controller_manager/list_controllers', controller_types= controller_types_list, cancel_service_list = cancel_list)

        executor = MultiThreadedExecutor(num_threads=2) #check number 
        executor.add_node(estop_node)

        #rclpy.spin(estop_node)
        try: 
            print("spinning")
            executor.spin()
            

        finally:
            executor.shutdown()
            estop_node.destroy_node()
        
    finally:
        #estop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()