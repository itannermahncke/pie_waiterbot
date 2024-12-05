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
    apriltag_ros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("apriltag_ros"), "launch"),
                "/v4l2_36h11.launch.yml",
            ]
        )
    )

    landmark_poses = os.path.join(
        get_package_share_directory("pie_waiterbot"), "landmark_poses.yaml"
    )

    robot_info = os.path.join(
        get_package_share_directory("pie_waiterbot"), "robot_info.yaml"
    )

    serial_config = os.path.join(
        get_package_share_directory("pie_waiterbot"), "serial.yaml"
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
                executable="pose_estimation",
                parameters=[landmark_poses],
            ),
            Node(
                package="pie_waiterbot",
                executable="map_maker",
                parameters=[landmark_poses],
            ),
            Node(
                package="pie_waiterbot",
                executable="goal_reach",
                parameters=[landmark_poses, robot_info],
            ),
            Node(
                package="pie_waiterbot",
                executable="serial_adapter",
                parameters=[serial_config],
            ),
            Node(
                package="pie_waiterbot",
                executable="fourbar_module",
                parameters=[robot_info],
            ),
            Node(
                package="pie_waiterbot",
                executable="path_planning",
                parameters=[landmark_poses],
            ),
        ]
    )
