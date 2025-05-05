from launch import LaunchDescription
from chonkur_deploy.launch_helpers import include_launch_file


def generate_launch_description():

    clr_launch = include_launch_file(
        package_name="clr_deploy",
        launch_file="control.launch.py",
        launch_arguments={
            "use_fake_hardware": "false",
            # The admittance controller will not work with the default humble controller, as the default IK
            # solver will include the full kinematic chain from the tool to the rail - which is 2 extra joints.
            # This is "fixed" moving forward by https://github.com/ros-controls/kinematics_interface/pull/73/files.
            # "enable_admittance": "true",
        }.items(),
    )

    camera_launch = include_launch_file(
        package_name="clr_deploy",
        launch_file="realsense_cameras.launch.py",
    )

    return LaunchDescription([clr_launch, camera_launch])
