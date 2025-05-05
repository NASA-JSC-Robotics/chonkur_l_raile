from launch import LaunchDescription
from chonkur_deploy.launch_helpers import include_launch_file


def generate_launch_description():

    lift_camera = include_launch_file(
        package_name="realsense2_camera",
        launch_file="rs_launch.py",
        launch_arguments={
            "camera_name": "lift_camera",
            "camera_namespace": "",
            "serial_no": "'207122078580'",
            "rgb_camera.profile": "1280,720,30",
            "initial_reset": "true",
            "pointcloud.enable": "false",
            "align_depth.enable": "true",
        }.items(),
    )

    wrist_camera = include_launch_file(
        package_name="chonkur_deploy",
        launch_file="realsense_cameras.launch.py",
    )

    return LaunchDescription([lift_camera, wrist_camera])
