import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
#from ur_dashboard_msgs.msg import SafetyMode
from controller_manager_msgs.srv import ListControllers
from action_msgs.srv import CancelGoal
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class ControllerCancelServiceExtractor(Node):
    def __init__(self):

        super().__init__('controller_cancel_service_extractor')

        # standard controller types for robot body and gripper
        self.body_type = "joint_trajectory_controller/JointTrajectoryController"
        self.gripper_type = "position_controllers/GripperActionController"
        
        # currently moveit_simple_controller_manager can only connect to FollowJointTrajectoryAction and GripperCommandAction servers.
        self.append_body_srv_name = "/follow_joint_trajectory/_action/cancel_goal"
        self.append_gripper_srv_name = "/gripper_cmd/_action/cancel_goal"

        self.cancel_service_list = []

    def add_cancel_service(self, list_ctrlrs_resp):
        for ctrlr in list_ctrlrs_resp.controller:
            if ctrlr.type == self.body_type:
                self.cancel_service_list.append(ctrlr.name + self.append_body_srv_name)

            elif ctrlr.type == self.gripper_type:
                self.cancel_service_list.append(ctrlr.name + self.append_gripper_srv_name)
        
        self.get_logger().info('this is the list of all cancel services: {}'.format(self.cancel_service_list))

class EStopSafety(Node):
    """ The EStopSafety Class is responsible for canceling all actions moving the robot after the e-stop has been triggered.

    Args:
        Node (Node): inherests from the Node class
    """
    def __init__(self, estop_topic, estop_values_list, estop_msg_type, estop_triggered, list_controllers_service_name = '/controller_manager/list_controllers'):
        """ the init function for the EStopSafety Class, which creates a subscriber to the EStop topic.

        Args:
            estop_topic (str): the topic EStopSafety is subscribed to which publishes the e-stop status
            estop_values_list (list): a list of e-stop values that, when published, should trigger a stop of the actions
            estop_msg_type (msg type): the type of message published by the estop
            estop_triggered (function): a function that checks if the estop has been triggered
            list_controllers_service_name (str, optional): the name of the service where it can retrive the list the controllers. Defaults to '/controller_manager/list_controllers'.
        """

        super().__init__('estop_safety')
        
        # create an instance of the ControllerCancelServiceExtractor
        self.extractor = ControllerCancelServiceExtractor()

        # the type of message published by the robot's estop
        self.estop_msg_type = estop_msg_type

        # a pass in the function that checks if an estop was triggered or not
        self.estop_triggered = estop_triggered

        # list of values that will trigger the estop
        self.estop_values_list = estop_values_list

        # service for listing the controllers (gets all configured controllers)
        self.list_controllers_service_name = list_controllers_service_name # set this to default, but it can be set later

        # create the two callback groups that the multi-threaded executr will run
        self.estop_cb_group = MutuallyExclusiveCallbackGroup()
        self.controllers_cb_group = MutuallyExclusiveCallbackGroup()

        # instantiate a subscriber to the estop topic
        self.estop_subscriber = self.create_subscription(
            self.estop_msg_type,
            estop_topic,
            self.estop_listener_callback,
            qos_profile = 10, # qos will keep the last 10 messages (this is the depth of the messaging history). Needs to match the qos_profile of the publisher to work
            callback_group = self.estop_cb_group)
        
    def estop_listener_callback(self, msg):
        """_summary_

        Args:
            msg (_type_): _description_
        """
        self.get_logger().info('I heard: "%s"' % msg)
        if(self.estop_triggered(msg, self.estop_values_list)):
            self.get_logger().info('e-stop was triggered')
            self.get_cancel_service_names()
            self.get_logger().info('pulled cancel service names')
            self.cancel_controllers()
            self.get_logger().info('cancelled controllers')
    
    def get_cancel_service_names(self):
        # create a service client to get the list of controllers
        self.list_ctrlrs_client = self.create_client(ListControllers, self.list_controllers_service_name, callback_group = self.controllers_cb_group)
        # wait for the corresponding server
        while not self.list_ctrlrs_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.list_ctrlrs_client.srv_name} is not available, waiting again...')
        # create a list controllers request client
        list_ctrlrs_req = ListControllers.Request()
        # get the list of controllers
        list_ctrlrs_resp = self.list_ctrlrs_client.call(list_ctrlrs_req)
        # send the list of controllers to the ControllerCancelServiceExtractor to extract the service names
        self.extractor.add_cancel_service(list_ctrlrs_resp)


    def cancel_controllers(self):
        # empty cancel goal request msg (works for both full robot and gripper)
        cancel_msg = CancelGoal.Request()
        # cancel all of the controllers actions
        self.get_logger().info("canceling controllers")
        for client_name in self.extractor.cancel_service_list:
            # create a service client
            cancel_client = self.create_client(CancelGoal, client_name, callback_group=self.controllers_cb_group)
            # wait for the corresponding server
            while not cancel_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            # send the request with the cancel message and get the response
            result = cancel_client.call(cancel_msg)
            self.get_logger().info('result {}'.format(result))