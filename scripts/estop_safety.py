#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from drt_ros2_control_tools.mina_support_classes_name_later import EStopSafety
from rclpy.executors import MultiThreadedExecutor 

def main(args=None):

    estop_values = [3,5,6,7,11,12,13]
    rclpy.init(args=args)

    estop_t = '/io_and_status_controller/safety_mode'
    list_controllers_service = '/controller_manager/list_controllers'
    #print("mina sanity check")

    try:
        estop_node = EStopSafety(estop_t, estop_values, list_controllers_service)

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