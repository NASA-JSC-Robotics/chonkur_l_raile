import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ur_dashboard_msgs.msg import SafetyMode
from controller_manager_msgs.srv import ListControllers
from action_msgs.srv import CancelGoal


class EStopSafety(Node):
    def __init__(self, estop_topic):
        super().__init__('estop_safety')
        self.subscription = self.create_subscription(
            SafetyMode,
            estop_topic,
            self.estop_listener_callback,
            10)
        self.subscription,  # prevent unused variable warning
        self.have_controllers = False
        
    def estop_listener_callback(self, stop_list, msg):
        # call get_controllers only once
        # need to call "cancel controllers" but only once
        # if estop + hit
        print("here")
        #self.cancel_controllers()
        self.get_logger().info('I heard: "%s"' % msg)
    
    def get_controllers(self, list_controllers_service_name):
        # this function returns the list of the controllers
        self.list_ctrlrs_client = self.create_client(ListControllers, list_controllers_service_name)

        while not self.list_ctrlrs_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.list_ctrlrs_client.srv_name} is not available, waiting again...')
        
        self.list_ctrlrs_req = ListControllers.Request()
        self.future = self.list_ctrlrs_client.call_async(self.list_ctrlrs_req)
        rclpy.spin_until_future_complete(self, self.future)
        self.spawned_ctrlrs = self.future.result().controller
        self.spawned_ctrlr_names = [ctrlr.name for ctrlr in self.spawned_ctrlrs]
        self.spawned_ctrlr_types = [ctrlr.type for ctrlr in self.spawned_ctrlrs]
        self.controller_names = []
        #print(self.spawned_ctrlr_names)
        #print(self.spawned_ctrlr_types)
        for i in range(0, len(self.spawned_ctrlr_names)):
            if(self.spawned_ctrlr_types[i] == "joint_trajectory_controller/JointTrajectoryController"):
                self.controller_names.append(self.spawned_ctrlr_names[i])
        #print("list of controller names .....................")
        #print(self.controller_names)

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
            cancel_client = self.create_client(CancelGoal, client_name)
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


