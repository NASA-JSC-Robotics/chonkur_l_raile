from launch import LaunchDescription
from chonkur_deploy.launch_helpers import include_launch_file


def generate_launch_description():

    chonkur_launch = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="control.launch.py",
        launch_arguments={
            "use_fake_hardware": "false",
        }.items(),
    )

    camera_launch = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="realsense_cameras.launch.py",
    )

    return LaunchDescription([chonkur_launch, camera_launch])
