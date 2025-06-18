from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="tf_prefix of the joint names, useful for \
        multi-robot setup. If changed, also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "finger_xacro",
            default_value="fngr_v6",
            choices=["fngr_nail_v2", "fngr_v2_m", "fngr_v6"],
            description="Chose which fingers are mounted to the gripper",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_ros2_control_tag",
            default_value="false",
            description="Whether to generate the ros2 control tag",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "clr",
            default_value="true",
            choices=["true", "false"],
            description="Choose whether to view CLR in the environment",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "shelf",
            default_value="false",
            choices=["true", "false"],
            description="Choose whether to view shelf in the environment",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ceiling_prop",
            default_value="false",
            choices=["true", "false"],
            description="Choose whether to view ceiling_prop in the environment",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hatch_4040",
            default_value="false",
            choices=["true", "false"],
            description="Choose whether to view hatch_4040 in the environment",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "express_rack",
            default_value="true",
            choices=["true", "false"],
            description="Choose whether to view express_rack in the environment",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hatch_4060",
            default_value="true",
            choices=["true", "false"],
            description="Choose whether to view hatch_4060 in the environment",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "second_trainer",
            default_value="false",
            choices=["true", "false"],
            description="Choose whether to view second_trainer in the environment",
        )
    )

    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    finger_xacro = LaunchConfiguration("finger_xacro")
    generate_ros2_control_tag = LaunchConfiguration("generate_ros2_control_tag")
    clr = LaunchConfiguration("clr")
    shelf = LaunchConfiguration("shelf")
    ceiling_prop = LaunchConfiguration("ceiling_prop")
    hatch_4040 = LaunchConfiguration("hatch_4040")
    express_rack = LaunchConfiguration("express_rack")
    hatch_4060 = LaunchConfiguration("hatch_4060")
    second_trainer = LaunchConfiguration("second_trainer")


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("clr_imetro_environments"), "urdf", "clr_trainer_multi_hatch.urdf.xacro"]
            ),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "headless_mode:=",
            headless_mode,
            " ",
            "finger_xacro:=",
            finger_xacro,
            " ",
            "generate_ros2_control_tag:=",
            generate_ros2_control_tag,
            " ",
            "clr:=",
            clr,
            " ",
            "shelf:=",
            shelf,
            " ",
            "ceiling_prop:=",
            ceiling_prop,
            " ",
            "hatch_4040:=",
            hatch_4040,
            " ",
            "express_rack:=",
            express_rack,
            " ",
            "hatch_4060:=",
            hatch_4060,
            " ",
            "second_trainer:=",
            second_trainer,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution([FindPackageShare("clr_imetro_environments"), "rviz", "view_robot.rviz"])

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace="imetro_environment",
        remappings=[],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        namespace="imetro_environment",
        remappings=[],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
