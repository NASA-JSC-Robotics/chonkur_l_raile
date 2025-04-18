from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "hande_dev_name",
            default_value="/tmp/hande",
            description="File descriptor that will be generated for the tool communication device. "
            "The user has be be allowed to write to this location. ",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ns",
            default_value="",
            description="Namespace for the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.102",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )

    hande_dev_name = LaunchConfiguration("hande_dev_name")
    ns = LaunchConfiguration("ns")
    robot_ip = LaunchConfiguration("robot_ip")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")

    hande_comm_node = Node(
        name="ur_tool_communication_hande",
        package="ur_robot_driver",
        executable="tool_communication.py",
        namespace=ns,
        output="both",
        parameters=[
            {
                "robot_ip": robot_ip,
                "tcp_port": tool_tcp_port,
                "device_name": hande_dev_name,
            }
        ],
    )

    nodes = [
        hande_comm_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
