from drt_ros2_control_tools.estop_safety_class import EStopSafety
from ur_dashboard_msgs.msg import SafetyMode


class CLREStopSafety(EStopSafety):
    """This class cancels the CLR's controllers when the EStop has been triggered.

    Args:
        EStopSafety (Node): This class cancels the general robot's controllers when the EStop has been triggered.
    """

    def __init__(self):
        """the constructor for the CLREStopSafety Class, which creates a subscriber to the CLR's EStop topic."""
        clr_estop_topic = "/io_and_status_controller/safety_mode"
        self.estop_values_list = [
            SafetyMode.PROTECTIVE_STOP,
            SafetyMode.SAFEGUARD_STOP,
            SafetyMode.SYSTEM_EMERGENCY_STOP,
            SafetyMode.ROBOT_EMERGENCY_STOP,
            SafetyMode.VIOLATION,
            SafetyMode.FAULT,
            SafetyMode.UNDEFINED_SAFETY_MODE,
            SafetyMode.AUTOMATIC_MODE_SAFEGUARD_STOP,
            SafetyMode.SYSTEM_THREE_POSITION_ENABLING_STOP,
        ]

        ur_estop_msg_type = SafetyMode
        stop_gripper = True
        list_controllers_service = "/controller_manager/list_controllers"

        super().__init__(
            estop_topic=clr_estop_topic,
            estop_msg_type=ur_estop_msg_type,
            stop_gripper=stop_gripper,
            list_controllers_service_name=list_controllers_service,
        )

    def estop_triggered(self, msg: SafetyMode):
        """Evaluate the estop SafetyMode msg to determine if the estop has been triggered. Returns true once the estop has been triggered and false all other times.


        Args:
            msg (SafetyMode): msg published on the estop topic.

        Returns:
            bool: if the estop has been triggered or not
        """
        return msg.mode in self.estop_values_list
