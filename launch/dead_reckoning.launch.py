from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    apriltag_poses = os.path.join(
        get_package_share_directory("pie_waiterbot"), "apriltag_poses.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="pie_waiterbot",
                executable="dead_reckoning",
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
