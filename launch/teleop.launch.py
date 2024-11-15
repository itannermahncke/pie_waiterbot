from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pie_waiterbot",
                executable="teleop_serial",
                parameters=[
                    {
                        "serial_port": "/dev/ttyACM0",
                    }
                ],
            ),
        ]
    )
