from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
<<<<<<< HEAD:launch/webcam.launch.py
    apriltag_ros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("apriltag_ros"), "launch"),
                "/v4l2_36h11.launch.yml",
            ]
        )
=======
    apriltag_poses = os.path.join(
        get_package_share_directory("pie_waiterbot"), "apriltag_poses.yaml"
>>>>>>> 73d7ebcd9608a8b4824720cf1ee0662a2fa11815:launch/dead_reckoning.launch.py
    )

    return LaunchDescription(
        [
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
            ),
            Node(
                package="pie_waiterbot",
<<<<<<< HEAD:launch/webcam.launch.py
                executable="webcam_driver",
=======
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
>>>>>>> 73d7ebcd9608a8b4824720cf1ee0662a2fa11815:launch/dead_reckoning.launch.py
            ),
        ]
    )
