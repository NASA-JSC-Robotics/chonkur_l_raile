#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ur_dashboard_msgs.srv import GetProgramState
from ur_dashboard_msgs.msg import ProgramState
from std_srvs.srv import SetBool
from rclpy.executors import MultiThreadedExecutor
import sys

# THIS FILE IS TO MOCK THE UR DASHBOARD CLIENT PROGRAM STATE IF YOU ARE TRYING TO TEST SOMETHING IN SIM


class ProgramStateServer(Node):
    def __init__(self):
        super().__init__("program_state_server")
        self.srv_get_state = self.create_service(
            GetProgramState, "/dashboard_client/program_state", self.get_program_state_callback
        )
        self.srv_set_bool = self.create_service(
            SetBool, "/fake_dashboard_client/set_program_state_running", self.set_program_state_callback
        )

        self.program_state = ProgramState()
        self.program_state.state = ProgramState.STOPPED

    def get_program_state_callback(self, request, response):
        response.state = self.program_state
        response.success = True
        return response

    def set_program_state_callback(self, request, response):
        # Change the program state based on the request data
        if request.data:
            self.program_state.state = ProgramState.PLAYING
            response.success = True
        else:
            self.program_state.state = ProgramState.PAUSED
            response.success = True

        self.get_logger().info(
            "SetBool request: " + str(request.data) + ", New program state: " + self.program_state.state
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    program_state_server = ProgramStateServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(program_state_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
