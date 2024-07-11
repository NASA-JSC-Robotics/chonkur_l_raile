import sys
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
#from ur_dashboard_msgs.msg import SafetyMode
from controller_manager_msgs.srv import ListControllers
from action_msgs.srv import CancelGoal
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class ControllerCancelServiceExtractor(Node):
    def __init__(self, include_gripper):

        super().__init__('controller_cancel_service_extractor')

        # key: standard controller types for robot body and gripper, value: server
        self.controller_dict = {
            "joint_trajectory_controller/JointTrajectoryController" : "/follow_joint_trajectory/_action/cancel_goal"
        }
        
        if include_gripper:
            self.controller_dict["position_controllers/GripperActionController"] = "/gripper_cmd/_action/cancel_goal"

        self.cancel_service_list = []

    def add_cancel_service(self, list_ctrlrs_resp):
        for ctrlr in list_ctrlrs_resp.controller: 
            if ctrlr.type in self.controller_dict:
                self.cancel_service_list.append(ctrlr.name + self.controller_dict[ctrlr.type]) #TODO print keys in cancrel msg later on
        self.get_logger().debug('this is the list of all cancel services for node "%s" : {}'.format(self.cancel_service_list) % self.get_name())

class EStopSafety(Node):
    """ The EStopSafety Class is responsible for canceling all actions moving the robot after the e-stop has been triggered.

    Args:
        Node (Node): inherests from the Node class
    """
    # TODO set types for all inputs
    #remove estop values list
    def __init__(self, estop_topic: String, estop_msg_type, include_gripper: bool = True, list_controllers_service_name: String = '/controller_manager/list_controllers'):
        """ the init function for the EStopSafety Class, which creates a subscriber to the EStop topic.

        Args:
            estop_topic (str): the topic EStopSafety is subscribed to which publishes the e-stop status
            estop_values_list (list): a list of e-stop values that, when published, should trigger a stop of the actions
            estop_msg_type (msg type): the type of message published by the estop
            estop_triggered (function): a function that checks if the estop has been triggered
            list_controllers_service_name (str, optional): the name of the service where it can retrive the list the controllers. Defaults to '/controller_manager/list_controllers'.
        """
        # start setting types in the init e.g. estop topic is  string

        super().__init__('estop_safety')
        
        # create an instance of the ControllerCancelServiceExtractor
        self.extractor = ControllerCancelServiceExtractor(include_gripper = include_gripper)

        # the type of message published by the robot's estop
        self.estop_msg_type = estop_msg_type

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
        
        # this is psudo code
    def estop_triggered(self): # this lives in drt but in the robot specific package I will need to interit this node and properly setup the ur specific stuff
        raise NotImplementedError

    def estop_listener_callback(self, msg):
        """this callback continuously listens to the robot's estop topic. Once the estop has been triggered, 
        it calls get_cancel_service_names(), which returns the list of joint trajectory and gripper controller services,
        then it calls cancel_controllers() which sends the cancel goal request to all the controllers. 
        This process is repeated continuously until a non-estop msg is sent on the estop topic.

        Args:
            msg (_type_): e-stop message
        """
        self.get_logger().debug('I heard: "%s" on the estop topic' % msg)
        if(self.estop_triggered(msg)):
            self.get_logger().info('e-stop was triggered for node "%s"' % self.get_name()) # publish the node name as well self.get node name
            self.get_cancel_service_names()
            self.cancel_controllers()
            self.get_logger().info('cancelled all joint trajectory and gripper controllers for node "%s"' % self.get_name()) 

    def get_cancel_service_names(self):
        # create a service client to get the list of controllers
        self.list_ctrlrs_client = self.create_client(ListControllers, self.list_controllers_service_name, callback_group = self.controllers_cb_group)
        # wait for the corresponding server
        #initial_time = time.time()
        counter = 0
        while not self.list_ctrlrs_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.list_ctrlrs_client.srv_name} is not available, waiting again...') 
            if counter >= 10:
                self.get_logger().error(f'unable to reach {self.list_ctrlrs_client.srv_name}') 
            counter +=1
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
        for client_name in self.extractor.cancel_service_list:
            # create a service client
            cancel_client = self.create_client(CancelGoal, client_name, callback_group=self.controllers_cb_group)
            # wait for the corresponding server
            counter = 0 # TODO rename to timeout_counter
            while not cancel_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{client_name} service not available, waiting again...') 
                if counter >= 10:
                    self.get_logger().error(f'unable to reach {client_name}') 
                counter += 1
            # send the request with the cancel message and get the response
            response = cancel_client.call(cancel_msg)
            self.get_logger().debug('response from the estop cancel request on "%s" {}'.format(response) %client_name)