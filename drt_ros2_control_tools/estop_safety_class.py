from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv._list_controllers import ListControllers_Request
from action_msgs.srv import CancelGoal
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from typing import TypeVar

EStopROSMsg = TypeVar('EStopROSMsg')

class ControllerCancelServiceExtractor(object):
    def __init__(self, include_gripper: bool, node: Node):
        """The constructor for the ControllerCancelServiceExtractor Class, which extracts the types of controllers to be cancelled.
        Args:
            include_gripper (bool): whether or not to include the gripper in the list of controllers to cancel.
        """
        # key: standard controller types for robot, value: the standard request name for the cancel service of that controller type (not fully rectified name) 
        self.controller_dict = {
            "joint_trajectory_controller/JointTrajectoryController" : "/follow_joint_trajectory/_action/cancel_goal"
        }
        
        if include_gripper:
            self.controller_dict["position_controllers/GripperActionController"] = "/gripper_cmd/_action/cancel_goal"

        self.cancel_service_list = []
        self.node = node

    def add_cancel_service(self, list_ctrlrs_resp: list[ListControllers_Request]):
        """Extracts the cancel services for the desired controller type.

        Args:
            list_ctrlrs_resp ([ListControllers_Request]): full list of robot controllers.
        """
        for ctrlr in list_ctrlrs_resp.controller: 
            if ctrlr.type in self.controller_dict:
                self.cancel_service_list.append(ctrlr.name + self.controller_dict[ctrlr.type])
        self.node.get_logger().debug('{node_name}: this is the list of all controllers to be cancelled {controllers}'.format(node_name = self.node.get_name(), controllers = self.cancel_service_list))

class EStopSafety(Node):
    """This class cancels the robot's controllers when the EStop has been triggered.

    Args:
        Node (Node): inherits the Node class
    """

    def __init__(self, estop_topic: str, estop_msg_type: EStopROSMsg, stop_gripper: bool = True, list_controllers_service_name: str = '/controller_manager/list_controllers'):
        """ the constructor for the EStopSafety Class, which creates a subscriber to the EStop topic.

        Args:
            estop_topic (str): the topic which publishes the e-stop status. 
            estop_msg_type (EStopRosMsg): the type of message published on the estop topic.
            stop_gripper (bool, optional): whether to stop the robot gripper or not. Defaults to True.
            list_controllers_service_name (str, optional): the name of the service where it can retrive the list the controllers. Defaults to '/controller_manager/list_controllers'.
        """
        # start setting types in the init e.g. estop topic is  string

        super().__init__('estop_safety')
        
        # create an instance of the ControllerCancelServiceExtractor
        self.extractor = ControllerCancelServiceExtractor(stop_gripper = stop_gripper, node = self)

        # the type of message published by the robot's estop
        self.estop_msg_type = estop_msg_type

        # service for listing the controllers (gets all configured controllers)
        self.list_controllers_service_name = list_controllers_service_name 

        # create the two callback groups that the multi-threaded executor will run
        self.estop_cb_group = MutuallyExclusiveCallbackGroup()
        self.controllers_cb_group = MutuallyExclusiveCallbackGroup()
        self.failed_cancel_cb_group = MutuallyExclusiveCallbackGroup()

        self.failed_cancel_list = []
        # timer
        timer_period = 1  # seconds
        self.failed_cancel_timer = self.create_timer(timer_period, self.failed_cancel_callback, callback_group= self.failed_cancel_cb_group)


        # instantiate a subscriber to the estop topic
        self.estop_subscriber = self.create_subscription(
            self.estop_msg_type,
            estop_topic,
            self.estop_listener_callback,
            qos_profile = 10, # qos will keep the last 10 messages (this is the depth of the messaging history). Needs to match the qos_profile of the publisher to work
            callback_group = self.estop_cb_group)
        
    def estop_triggered(self, msg: EStopROSMsg) -> bool:
        """Evaluate the estop ros msg to determine if the estop has been triggered. Returns true once the estop has been triggered and false all other times. To be overriden by the robot specific method.

        Raises:
            NotImplementedError

        Returns:
            bool: if the estop has been triggered or not
        """
        raise NotImplementedError("estop_triggered function not implemented for the robot")

    def estop_listener_callback(self, msg: EStopROSMsg):
        """This callback continuously listens to the robot's estop topic. Once the estop has been triggered, 
        it calls get_cancel_service_names(), which returns the list of joint trajectory and gripper controller services,
        then it calls cancel_controllers() which sends the cancel goal request to all the controllers. 
        This process is repeated continuously until a non-estop msg is sent on the estop topic.

        Args:
            msg (EStopRosMsg): message published on the estop topic
        """
        self.get_logger().debug('{node_name} Estop msg: {e_msg}'.format(node_name = self.get_name(), e_msg = msg))
        if(self.estop_triggered(msg)):
            self.get_logger().info('e-stop was triggered for node "%s"' % self.get_name()) 
            self.get_cancel_service_names()
            self.cancel_controllers()
            self.get_logger().info('{node_name} cancelled all {keys} controllers'.format(node_name = self.get_name(), keys = self.extractor.controller_dict.keys()))

    def get_cancel_service_names(self):
        """Gets the names of the services for the controllers that will be cancelled. 
        """
        # create a service client to get the list of controllers
        self.list_ctrlrs_client = self.create_client(ListControllers, self.list_controllers_service_name, callback_group = self.controllers_cb_group)
        # wait for the corresponding server
        timeout_counter = 0
        while not self.list_ctrlrs_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.list_ctrlrs_client.srv_name} is not available, waiting again...') 
            if timeout_counter >= 10:
                self.get_logger().error(f'unable to reach {self.list_ctrlrs_client.srv_name}. to get the list of active controllers that need to be cancelled. Active controllers were not stopped, so there may be unexpected movement. Be careful when re-enabling the robot.')
            timeout_counter += 1
        # create a list controllers request client
        list_ctrlrs_req = ListControllers.Request()
        # get the list of controllers
        list_ctrlrs_resp = self.list_ctrlrs_client.call(list_ctrlrs_req)
        # send the list of controllers to the ControllerCancelServiceExtractor to extract the service names
        self.extractor.add_cancel_service(list_ctrlrs_resp)

    def cancel_controllers(self):
        """Send a cancel message to all the controller cancel services. 
        """
        # empty cancel goal request msg (works for both full robot and gripper)
        cancel_msg = CancelGoal.Request()
        # cancel all of the controllers actions
        for client_name in self.extractor.cancel_service_list:
            # create a service client
            cancel_client = self.create_client(CancelGoal, client_name, callback_group=self.controllers_cb_group)
            # wait for the corresponding server
            timeout_counter = 0
            failed_find_service = False 
            while not cancel_client.wait_for_service(timeout_sec=1.0) and not failed_find_service:
                self.get_logger().info(f'{client_name} service not available, waiting again...') 
                if timeout_counter >= 2:
                    if client_name not in self.failed_cancel_list:
                        self.failed_cancel_list.append(client_name)
                    self.get_logger().error(f'unable to reach {client_name} to cancel its trajectory, so there may be unexpected movement. Be careful when re-enabling the robot.') 
                    self.failed_cancel_callback()
                    failed_find_service = True
                timeout_counter += 1
            # send the request with the cancel message and get the response
            if not failed_find_service:
                # execute the service
                response = cancel_client.call(cancel_msg)
                
    def failed_cancel_callback(self):
        for client_name in self.failed_cancel_list:
            self.get_logger().error(f'unable to reach {client_name} to cancel its trajectory, so there may be unexpected movement. Be careful when re-enabling the robot.') 
                    