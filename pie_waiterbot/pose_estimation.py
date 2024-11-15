import rclpy
from rclpy.node import Node

import constants

import apriltag

from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, TransformStamped, Pose
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class PoseEstimationNode(Node):
    """
    Compute best possible pose estimate.

    Currently bases pose estimation off of AprilTag homography. Eventually
    will utilize sensor fusion from encoder data and IMU data.
    """

    def __init__(self):
        """
        Initialize an instance of the PoseEstimationNode.
        """
        super().__init__("pose_estimation")

        # transformation management
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_listener = TransformListener(self)
        self.make_apriltag_transformations()

        # subscribers
        self.detection_subscriber = self.create_subscription(
            "detections", AprilTagDetectionArray, self.detection_callback, 10
        )
        self.transform_subscriber = self.create_subscription(
            "tf", TFMessage, self.tf_callback, 10
        )

        # pose publisher
        self.pose_publisher = self.create_publisher(Pose, "pose_estimate", 10)

    def make_apriltag_transformations(self):
        """
        Define all of the static transformations between each AprilTag and the
        world coordinate frame.
        """
        apriltags = self.get_parameter("apriltags")
        for apriltag_id in apriltags:
            translation = self.get_parameter(f"{apriltag_id}_translation")
            rotation = self.get_parameter(f"{apriltag_id}_rotation")
            tf = self.make_static_transform(apriltag_id, translation, rotation)
            self.tf_static_broadcaster.sendTransform(tf)

    def make_static_transform(self, id, translation, rotation):
        """
        Helper function to create a static transform between a single frame
        and the world frame.
        """
        tf = TransformStamped()
        header = Header()
        transform = Transform()

        header.frame_id = 1

        transform.translation = translation
        transform.rotation = rotation

        tf.header = header
        tf.transform = transform
        tf.child_frame_id = id

        return tf

    def tf_callback(self, tfmessage: TFMessage):
        tf: TransformStamped
        for tf in tfmessage.transforms:
            if tf.child_frame_id in constants.apriltag_poses.keys():
                self.get_parameter(f"")
                at_world_coords = constants.apriltag_poses[tf.child_frame_id]
                tf.transform


def main(args=None):
    rclpy.init(args=args)
    pose_estimation = PoseEstimationNode()
    rclpy.spin(pose_estimation)
    pose_estimation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
