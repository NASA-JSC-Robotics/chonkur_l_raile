from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from controller_manager_msgs.srv import ListControllers, SwitchController
from std_srvs.srv import SetBool

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ControllerStopperBase(Node):
    """This class cancels the stopping and starting controllers based on some event (implemented by a derived class).

    Args:
        Node (Node): inherits the Node class
    """

    def __init__(self, node_name='controller_stopper', controller_manager_name='/controller_manager', servo_node_name=''):
        """ the constructor for the ControllerStopperBase Class, which creates can handle stopping and starting controllers.

        Args:
            node_name (str, optional): the name of the node that will be created. Defaults to 'controller_stopper'
            controller_manager_name (str, optional): the name of the controller manager node (with namespace) to use for service call prefixes. defaults to '/controller_manager'
            servo_node_name (str, optional): the name of servo node (leave blank if there is none). defaults to ''
        """
        # initialize parent node name as 
        super().__init__(node_name)

        # use the same callback group for controller manager services because we don't want to be listing controllers as we are switching them
        self.cm_cb_group = MutuallyExclusiveCallbackGroup()

        # setup services for switch and list controllers
        self.switch_controllers_srv = self.create_client(SwitchController, controller_manager_name + '/switch_controller',callback_group=self.cm_cb_group)
        self.list_controllers_srv = self.create_client(ListControllers, controller_manager_name + '/list_controllers', callback_group=self.cm_cb_group)

        # wait until all required services are available from controller_manager
        self.get_logger().info("Waiting for switch controller service to come up on controller_manager/switch_controller")
        self.switch_controllers_srv.wait_for_service()
        self.get_logger().info("Service available")
        self.get_logger().info("Waiting for list controllers service to come up on controller_manager/list_controllers")
        self.list_controllers_srv.wait_for_service()
        self.get_logger().info("Service available")

        # if the user doesn't provide the name of the servo node, assume we don't care
        self.servo_node_name = servo_node_name
        self.manage_servo = (self.servo_node_name != '')
        self.servo_node_found = True

        # only try to hook up to the node if we are managing servo as well
        if self.manage_servo:
            # use a different callback group for servo pausing/unpausing so that can run in parallel
            self.servo_cb_group = MutuallyExclusiveCallbackGroup()
            self.pause_servo_srv = self.create_client(SetBool, self.servo_node_name + '/pause_servo', callback_group=self.servo_cb_group)

        # consistent_controllers_descriptor = ParameterDescriptor(,description='This parameter is mine!')

        # get parameter of list of strings for consistent_controllers (default to nothing)
        self.declare_parameter(
            'consistent_controllers', 
            [''], 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='controllers which will remain active even when the pendant is not running a program'
            )
        )
        self.consistent_controllers = self.get_parameter('consistent_controllers').value
        if self.consistent_controllers == ['']: self.consistent_controllers == []

        print(f"{self.consistent_controllers}")

        # variable to keep track of which controllers have been stopped so they can be restarted
        self.stopped_controllers = []

        # variable to keep track of whether we are in a state where we have restarted controllers
        self.controllers_active = True

    def servo_node_exists(self):
        """Returns true if the servo node was found in active ROS session. And logs a message if the state
        changes from one to another.
        """
        # list available nodes and look for the servo node in it
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        for node, _ in node_names_and_namespaces:
            if node == self.servo_node_name:
                # if we found the node, but we didn't have it previously, let the user know we are now tracking it
                if not self.servo_node_found:
                    self.get_logger().info(f'Servo node "{self.servo_node_name}" is now being handled.')
                self.servo_node_found = True
                # exit early
                return True

        # if last cycle we found the node, but now we can't find it, let the user know that we are no longer tracking
        if self.servo_node_found:
            self.get_logger().info(f'{bcolors.WARNING}Was asked to monitor the servo node "{self.servo_node_name}" but it was not found{bcolors.ENDC}')
        self.servo_node_found = False

        return False

    def stop_controllers(self):
        """Deactivates all of the controllers that are not listed as consistent.
        This can be called several times without causing issues. This is useful because
        some controllers may be turned back on in some cases, like if a trajectory is accidentally
        performed while the robot is disabled
        """
        # only save the controllers to restart if this is the first cycle when the controllers were active
        # this makes it so we don't get conflicts if we tried to load another controller that needed the same
        # interfaces as one that was already active
        save_controllers = self.controllers_active

        # request to get switch controller, list controller, and pause servo
        switch_controller_request = SwitchController.Request()
        list_controllers_request = ListControllers.Request()
        pause_servo_request = SetBool.Request()

        # set strictness to strict to show that this fails if anything went wrong
        switch_controller_request.strictness = SwitchController.Request.STRICT
        switch_controller_request.timeout.nanosec = 500000000 # 0.5s

        # list the controllers that are running
        list_controllers_response = self.list_controllers_srv.call(list_controllers_request)

        # each new time this is called, we will repopulate which controllers we are stopping
        controllers_to_stop = []

        # add controllers that are active and not consistent to the list to stop
        for controller in list_controllers_response.controller:
            if controller.state == 'active':
                if controller.name not in self.consistent_controllers:
                    controllers_to_stop.append(controller.name)

        # if there are any to stop, call switch_controllers to stop them
        if controllers_to_stop:
            switch_controller_request.deactivate_controllers = controllers_to_stop
            switch_controllers_response = self.switch_controllers_srv.call(switch_controller_request)
            if not switch_controllers_response.ok:
                self.get_logger().error('Could not deactivate requested controllers')
            else:
                self.controllers_active = False

        # save the stopped controllers only if we are in a state where the controllers were active
        if save_controllers:
            self.stopped_controllers = controllers_to_stop

        # if we are managing the servo controller, also pause the servo node
        if self.manage_servo:
            if self.servo_node_exists():
                pause_servo_request.data = True
                pause_servo_response = self.pause_servo_srv.call(pause_servo_request)
                if not pause_servo_response.success:
                    self.get_logger().error('Could not pause servo node')

    def start_controllers(self):
        """Reactivates all of the controllers that have been stopped by this node.
        """
        # initialize service requests
        switch_controller_request = SwitchController.Request()
        pause_servo_request = SetBool.Request()

        # set strictness to strict to show that this fails if anything went wrong
        switch_controller_request.strictness = SwitchController.Request.STRICT
        switch_controller_request.timeout.nanosec = 500000000 # 0.5s

        # only run this if we have logged that some controllers have been stopped
        if self.stopped_controllers:
            switch_controller_request.activate_controllers = self.stopped_controllers

            switch_controllers_response = self.switch_controllers_srv.call(switch_controller_request)
            if not switch_controllers_response.ok:
                self.get_logger().error('Could not activate requested controllers')
            else:
                self.controllers_active = True

        # if we are managing the servo controller, restart the servo node
        if self.manage_servo:
            if self.servo_node_exists():
                pause_servo_request.data = False
                pause_servo_response = self.pause_servo_srv.call(pause_servo_request)
                if not pause_servo_response.success:
                    self.get_logger().error('Could not unpause servo node')

        # clear stopped controllers for the next round to populate
        self.stopped_controllers.clear()