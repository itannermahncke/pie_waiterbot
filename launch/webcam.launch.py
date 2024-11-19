from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    apriltag_ros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("apriltag_ros"), "launch"),
                "/v4l2_36h11.launch.yml",
            ]
        )
    )

    return LaunchDescription(
        [
            apriltag_ros,
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
            ),
            Node(
                package="pie_waiterbot",
                executable="webcam_driver",
            ),
        ]
    )
