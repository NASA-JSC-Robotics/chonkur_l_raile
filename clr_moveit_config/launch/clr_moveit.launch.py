
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from ur_moveit_config.launch_common import load_yaml
from moveit_configs_utils import MoveItConfigsBuilder

import os

def generate_launch_description():

    description_package = 'clr_description'
    description_file = 'clr.urdf.xacro'
    moveit_config_package = 'clr_moveit_config'
    moveit_config_srdf_file = 'clr.srdf.xacro'

    moveit_config = (
        MoveItConfigsBuilder("clr")
        .robot_description(file_path="/home/ndunkelb/clr_ws/install/clr_description/share/clr_description/urdf/clr.urdf.xacro")
        .robot_description_semantic(file_path="config/" + moveit_config_srdf_file)
        .robot_description_kinematics(file_path="config/" + "kinematics.yaml")
        .to_moveit_configs()
    )

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="tf_prefix of the joint names, useful for \
        multi-robot setup. If changed, also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    tf_prefix = LaunchConfiguration("tf_prefix")

    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
    #         " ",        
    #         "use_fake_hardware:=",
    #         use_fake_hardware,
    #         " ",
    #         "tf_prefix:=",
    #         tf_prefix,
    #         " ",
    #     ]
    # )
    # robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    # robot_description_semantic_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare(moveit_config_package), "srdf", moveit_config_srdf_file]
    #         ), 
    #         " ", 
    #         "name:=",
    #         "clr",
    #         " ",               
    #     ]
    # )
    # robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # Planning Configuration
    # ompl_planning_pipeline_config = {
    #     "move_group": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 0.1,
    #     }
    # }
    # ompl_planning_yaml = load_yaml(moveit_config_package, "config/ompl_planning.yaml")
    # ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # # Trajectory Execution Configuration
    # moveit_controllers = {
    #     "moveit_controller_manager": "moveit_ros_control_interface/Ros2ControlManager",
    # }

    # trajectory_execution = {
    #     "moveit_manage_controllers": False,
    #     "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    #     "trajectory_execution.allowed_goal_duration_margin": 0.5,
    #     "trajectory_execution.allowed_start_tolerance": 0.01,
    # }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # warehouse_ros_config = {
    #     "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
    #     "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
    # }


    # # joint limits
    # joint_limits_yaml = load_yaml(moveit_config_package, "config/joint_limits.yaml")
    # joint_limits = {'robot_description_planning': joint_limits_yaml}

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            # robot_description,
            # robot_description_semantic,
            # ompl_planning_pipeline_config,
            # robot_description_kinematics,
            # joint_limits,
            # warehouse_ros_config,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            robot_description_kinematics,
            planning_scene_monitor_parameters,
            # warehouse_ros_config
        ],
    )    



    nodes_to_start = [
        move_group_node, 
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)    
