#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from clr_control.clr_estop_safety_class import CLREStopSafety

def main(args=None):
    
    rclpy.init(args=args)

    try:
        clr_estop_node = CLREStopSafety()

        executor = MultiThreadedExecutor(num_threads=2) #check number 
        executor.add_node(clr_estop_node)

        try: 
            print("spinning")
            executor.spin()         

        finally:
            executor.shutdown()
            clr_estop_node.destroy_node()
        
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()