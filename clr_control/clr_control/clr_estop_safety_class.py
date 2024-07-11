from drt_ros2_control_tools.estop_safety_class import EStopSafety
from ur_dashboard_msgs.msg import SafetyMode

class CLREStopSafety(EStopSafety):
    def __init__(self):
        clr_estop_topic = '/io_and_status_controller/safety_mode'
        self.estop_values_list = [3,5,6,7,11,12,13]
        ur_estop_msg_type = SafetyMode
        include_gripper = True
        list_controllers_service = '/controller_manager/list_controllers'

        super().__init__(estop_topic = clr_estop_topic, estop_msg_type = ur_estop_msg_type, include_gripper = include_gripper, list_controllers_service_name = list_controllers_service)

    def estop_triggered(self, msg):
        return msg.mode in self.estop_values_list