#!/usr/bin/env python3
import rclpy
from ur_dashboard_msgs.srv import GetProgramState
from ur_dashboard_msgs.msg import ProgramState
from drt_ros2_control_tools.controller_stopper_base import ControllerStopperBase
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class ChonkurControllerStopper(ControllerStopperBase):
    """This class cancels the stopping and starting controllers based on some event (implemented by a derived class).

    Args:
        ControllerStopperBase (ControllerStopperBase): inherits the ControllerStopperBase class
    """

    def __init__(self):
        """ the constructor for the ChonkurControllerStopper Class, which creates can handle stopping/starting 
        controllers, and pausing/unpausing servo when the program stops running.
        """

        # create instance of ControllerStopperBase with node name, and telling it we do want it to manage the servo node
        super().__init__(node_name='chonkur_controller_stopper', servo_node_name='servo_node')

        self.get_state_cb_group = ReentrantCallbackGroup()
        self.get_program_state_srv = self.create_client(GetProgramState,'/ur_dashboard_client/get_program_state', callback_group=self.get_state_cb_group)

        self.get_logger().info("Waiting for service to come up on /ur_dashboard_client/get_program_state")
        self.get_program_state_srv.wait_for_service()

        # timer at 1 second loop to check controller status and cancel 
        self.timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(1.0, self.timer_callback, callback_group=self.timer_cb_group)

        self.get_logger().info("Chonkur Controller Stopper is running!")

    def timer_callback(self):
        self.get_logger().info('here')
        request = GetProgramState.Request()
        result = self.get_program_state_srv.call(request)
        self.get_logger().info('here2')

        if not result.success:
            self.get_logger().error('was not able to get the state of the program')
            return # dashboard client publishes its own failure message

        # if we are either paused or stopped ,we treat that as not running
        self.robot_running = result.state.state == ProgramState.PLAYING

        # if we just transitioned to a running state, and the controllers weren't active, 
        # start the controllers
        if (self.robot_running and not self.controllers_active):
            # stop controllers first to get rid of anything that may have happened recently 
            self.stop_controllers()
            # start controllers 
            self.start_controllers()
        # if robot is either paused or stopped, consistently stop controllers to cancel anything that may have started
        elif not self.robot_running:
            self.stop_controllers()

def main(args=None):
    rclpy.init(args=args)
    chonkur_controller_stopper = ChonkurControllerStopper()
    executor = MultiThreadedExecutor()
    executor.add_node(chonkur_controller_stopper)
    rclpy.spin(chonkur_controller_stopper)
    chonkur_controller_stopper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()