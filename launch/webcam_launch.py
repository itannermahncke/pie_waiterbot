from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    apriltag_ros = IncludeLaunchDescription(
        package="apriltag_ros", launch="launch/v4l2_36h11.launch.yml"
    )

    apriltag_poses = os.path.join(
        get_package_share_directory("pie_waiterbot"), "config", "apriltag_poses.yaml"
    )

    return LaunchDescription(
        [
            apriltag_ros,
            Node(
                package="pie_waiterbot",
                executable="webcam_driver",
            ),
            Node(
                package="pie_waiterbot",
                executable="pose_estimation",
            ),
            Node(
                package="pie_waiterbot",
                executable="goal_driver",
                parameters=[apriltag_poses],
            ),
            Node(
                package="pie_waiterbot",
                executable="serial_adapter",
                parameters=[{"serial_port": "/dev/ttyACM0"}],
            ),
        ]
    )
