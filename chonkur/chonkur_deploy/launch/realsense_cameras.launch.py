from launch import LaunchDescription
from chonkur_deploy.launch_helpers import include_launch_file


def generate_launch_description():

    wrist_camera = include_launch_file(
        package_name="realsense2_camera",
        launch_file="rs_launch.py",
        launch_arguments={
            "camera_name": "wrist_mounted_camera",
            "camera_namespace": "",
            "serial_no": "'938422070949'",
            "rgb_camera.profile": "1280,720,30",
            "initial_reset": "true",
            "pointcloud.enable": "false",
            "align_depth.enable": "true",
        }.items(),
    )

    return LaunchDescription([wrist_camera])
