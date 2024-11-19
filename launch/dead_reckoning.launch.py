from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    apriltag_poses = os.path.join(
        get_package_share_directory("pie_waiterbot"), "apriltag_poses.yaml"
    )

    serial_config = os.path.join(
        get_package_share_directory("pie_waiterbot"), "serial.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
            ),
            Node(
                package="pie_waiterbot",
                executable="map_maker",
                parameters=[apriltag_poses],
            ),
            Node(
                package="pie_waiterbot",
                executable="dead_reckoning",
            ),
            Node(
                package="pie_waiterbot",
                executable="goal_driver",
                parameters=[apriltag_poses, {"goal_id": "test1"}],
            ),
            Node(
                package="pie_waiterbot",
                executable="serial_adapter",
                parameters=[serial_config],
            ),
        ]
    )
