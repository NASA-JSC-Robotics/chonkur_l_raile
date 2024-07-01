#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from drt_ros2_control_tools.mina_support_classes_name_later import EStopSafety
############### for testing
def main(args=None):
    rclpy.init(args=args)

    estop_t = '/io_and_status_controller/safety_mode'
    minimal_subscriber = EStopSafety(estop_t)

    list_controllers_service = '/controller_manager/list_controllers'

    # getting controllers
    print("getting controllers")
    minimal_subscriber.get_controllers(list_controllers_service)
    # cancelling
    #print("cancelling controllers")
    #minimal_subscriber.cancel_controllers()

    ######################
    param_topic_controller_manager = '/controller_manager'
    
    
    #cancel_msg = CancelGoal.Request()
    #print(cancel_msg.goal_info)


    # cancel_client = minimal_subscriber.create_client(CancelGoal, '/joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal')
    # while not cancel_client.wait_for_service(timeout_sec=1.0):
    #         minimal_subscriber.get_logger().info('service not available, waiting again...')
    
    # cancel_client.call(cancel_msg)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()