import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ur_dashboard_msgs.msg import SafetyMode
from controller_manager_msgs.srv import ListControllers
from action_msgs.srv import CancelGoal
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup



class EStopSafety(Node):
    def __init__(self, estop_topic, stop_values, cancel_service_list, list_controllers_service_name = '/controller_manager/list_controllers', controller_types = ["joint_trajectory_controller/JointTrajectoryController"]):
        super().__init__('estop_safety')

        self.estop_values_list = stop_values
        self.list_controllers_service_name = list_controllers_service_name # set this to default, but it can be set later

        self.estop_cb_group = MutuallyExclusiveCallbackGroup()
        self.controllers_cb_group = MutuallyExclusiveCallbackGroup()

        self.accepted_controller_types = controller_types
        # string that will append to the controller name
        self.joint_trajectory_cancel_service = cancel_service_list

        self.estop_subscriber = self.create_subscription(
            SafetyMode,
            estop_topic,
            self.estop_listener_callback,
            qos_profile = 10, # qos will keep the last 10 messages (this is the depth of the messaging history). Needs to match the qos_profile of the publisher to work
            callback_group = self.estop_cb_group)
        
    def estop_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        if (msg.mode in self.estop_values_list): # need to abstract this out since mode is specific to UR
            # if the safety messages are triggered, get the controllers and cancel them
            self.get_controllers()
            self.cancel_controllers()
            print("done cancelling actions")
    
    def get_controllers(self):
        # this function returns the list of the controllers
        self.list_ctrlrs_client = self.create_client(ListControllers, self.list_controllers_service_name, callback_group = self.controllers_cb_group)

        while not self.list_ctrlrs_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.list_ctrlrs_client.srv_name} is not available, waiting again...')
    
        list_ctrlrs_req = ListControllers.Request()
        list_ctrlrs_resp = self.list_ctrlrs_client.call(list_ctrlrs_req)
        spawned_ctrlr_names = [ctrlr.name for ctrlr in list_ctrlrs_resp.controller]
        spawned_ctrlr_types = [ctrlr.type for ctrlr in list_ctrlrs_resp.controller]
        print(spawned_ctrlr_names)
        print("types -----------------: ", spawned_ctrlr_types)
        self.controller_names = []
        for i in range(0, len(spawned_ctrlr_names)):
            if(spawned_ctrlr_types[i] in self.accepted_controller_types):
                self.controller_names.append(spawned_ctrlr_names[i])
        print("list of controller names .....................")
        print(self.controller_names)


    def cancel_controllers(self):
        # empty cancel goal request msg (works for both full robot and gripper)
        cancel_msg = CancelGoal.Request()

        # cancel all of the controllers actions
        self.get_logger().info("canceling controllers")
        for name in self.controller_names:
            client_name = name + self.joint_trajectory_cancel_service[1] # fix
            self.get_logger().info('client_name {}'.format(client_name))
        #     print("this is the client name: ", client_name)

        #     #print("this is the client name " + client_name)
        #     # instantiate a client
            cancel_client = self.create_client(CancelGoal, client_name, callback_group=self.controllers_cb_group)
            while not cancel_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            result = cancel_client.call(cancel_msg)
            self.get_logger().info('result {}'.format(result))



# __init__
#     subscriber = estop_cb
#     service call - get controllers
#     self.controllers = blah

# def estop_cb()
#     if estop_high:
#         make cancel service requests


