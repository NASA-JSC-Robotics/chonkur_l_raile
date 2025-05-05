from launch import LaunchDescription
from chonkur_deploy.launch_helpers import include_launch_file


def generate_launch_description():

    clr_launch = include_launch_file(
        package_name="clr_deploy",
        launch_file="control.launch.py",
        launch_arguments={
            "use_fake_hardware": "true",
        }.items(),
    )

    return LaunchDescription([clr_launch])
