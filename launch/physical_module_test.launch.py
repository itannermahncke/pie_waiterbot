"""
Launch module and serial adapter only.
"""

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    robot_info = os.path.join(
        get_package_share_directory("pie_waiterbot"), "robot_info.yaml"
    )
    serial_config = os.path.join(
        get_package_share_directory("pie_waiterbot"), "serial_config.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="pie_waiterbot",
                executable="fourbar_module",
                parameters=[robot_info],
            ),
            Node(
                package="pie_waiterbot",
                executable="serial_adapter",
                parameters=[serial_config],
            ),
        ]
    )
