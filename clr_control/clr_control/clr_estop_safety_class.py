from drt_ros2_control_tools.estop_safety_class import EStopSafety
from ur_dashboard_msgs.msg import SafetyMode

class CLREStopSafety(EStopSafety):
    def __init__(self):
        clr_estop_topic = '/io_and_status_controller/safety_mode'
        ur_estop_values = [3,5,6,7,11,12,13]
        ur_estop_msg_type = SafetyMode
        list_controllers_service = '/controller_manager/list_controllers'

        super().__init__(estop_topic = clr_estop_topic, estop_values_list = ur_estop_values, estop_msg_type = ur_estop_msg_type, list_controllers_service_name = list_controllers_service)

    def estop_triggered(self, msg, estop_values_list):
        return msg.mode in estop_values_list