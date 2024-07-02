import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ur_dashboard_msgs.msg import SafetyMode
from controller_manager_msgs.srv import ListControllers
from action_msgs.srv import CancelGoal
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup



class EStopSafety(Node):
    def __init__(self, estop_topic, list_controllers_service_name):
        super().__init__('estop_safety')
        self.estop_values = [7,11]
        self.list_controllers_service_name = list_controllers_service_name

        self.subscription = self.create_subscription(
            SafetyMode,
            estop_topic,
            self.estop_listener_callback,
            10)
        self.subscription,  # prevent unused variable warning
        self.pull_controllers_cb_group = MutuallyExclusiveCallbackGroup()
        self.cancel_controllers_cb_group = ReentrantCallbackGroup() 
        
    def estop_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        if (msg.mode in self.estop_values):
            # if the safety messages are triggered, get the controllers and cancel them
            self.get_controllers()
            self.cancel_controllers()
            print("done cancelling actions")
    
    def get_controllers(self):
        # this function returns the list of the controllers
        self.list_ctrlrs_client = self.create_client(ListControllers, self.list_controllers_service_name, callback_group= self.pull_controllers_cb_group)

        while not self.list_ctrlrs_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.list_ctrlrs_client.srv_name} is not available, waiting again...')
        
        self.list_ctrlrs_req = ListControllers.Request()
        self.future = self.list_ctrlrs_client.call_async(self.list_ctrlrs_req)
        rclpy.spin_until_future_complete(self, self.future)
        self.spawned_ctrlrs = self.future.result().controller
        self.spawned_ctrlr_names = [ctrlr.name for ctrlr in self.spawned_ctrlrs]
        self.spawned_ctrlr_types = [ctrlr.type for ctrlr in self.spawned_ctrlrs]
        self.controller_names = []
        for i in range(0, len(self.spawned_ctrlr_names)):
            if(self.spawned_ctrlr_types[i] == "joint_trajectory_controller/JointTrajectoryController"):
                self.controller_names.append(self.spawned_ctrlr_names[i])
        print("list of controller names .....................")
        print(self.controller_names)

    def cancel_controllers(self):
        # empty cancel goal request msg (works for both full robot and gripper)
        cancel_msg = CancelGoal.Request()

        # string that will append to the controller name
        joint_trajectory_cancel_service = "/follow_joint_trajectory/_action/cancel_goal"

        # cancel all of the controllers actions
        for name in self.controller_names:
            client_name = name + joint_trajectory_cancel_service

            #print("this is the client name " + client_name)
            # instantiate a client
            cancel_client = self.create_client(CancelGoal, client_name, callback_group=self.cancel_controllers_cb_group)
            while not cancel_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
                cancel_client.call(cancel_msg)



# __init__
#     subscriber = estop_cb
#     service call - get controllers
#     self.controllers = blah

# def estop_cb()
#     if estop_high:
#         make cancel service requests


