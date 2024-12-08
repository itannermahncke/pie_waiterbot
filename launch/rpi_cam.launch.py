from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    landmark_poses = os.path.join(
        get_package_share_directory("pie_waiterbot"), "landmark_poses.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                arguments=[
                    "-r",
                    "image_rect:=/image_raw",
                    "-r",
                    "camera_info:=/camera_info",
                ],
            ),
            Node(
                package="pie_waiterbot",
                executable="map_maker",
                parameters=[landmark_poses],
            ),
            Node(
                package="pie_waiterbot",
                executable="pose_estimation",
                parameters=[landmark_poses],
            ),
        ]
    )
