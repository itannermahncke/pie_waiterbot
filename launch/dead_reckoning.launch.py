from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    apriltag_poses = os.path.join(
        get_package_share_directory("pie_waiterbot"), "apriltag_poses.yaml"
    )

    robot_info = os.path.join(
        get_package_share_directory("pie_waiterbot"), "robot_info.yaml"
    )

    serial_config = os.path.join(
        get_package_share_directory("pie_waiterbot"), "serial.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="pie_waiterbot",
                executable="map_maker",
                parameters=[apriltag_poses],
            ),
            Node(
                package="pie_waiterbot",
                executable="dead_reckoning",
                parameters=[robot_info],
            ),
            Node(
                package="pie_waiterbot",
                executable="goal_driver",
                parameters=[apriltag_poses, robot_info],
            ),
            Node(
                package="pie_waiterbot",
                executable="serial_adapter",
                parameters=[serial_config],
            ),
        ]
    )