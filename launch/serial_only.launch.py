"""
Launch serial node only.
"""

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    serial_config = os.path.join(
        get_package_share_directory("pie_waiterbot"), "serial.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="pie_waiterbot",
                executable="serial_adapter",
                parameters=[serial_config],
            ),
        ]
    )
