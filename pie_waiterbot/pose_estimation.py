import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Pose
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations as tf


class PoseEstimationNode(Node):
    """
    Compute best possible pose estimate using transformation between the robot
    camera and any visible AprilTags.
    """

    def __init__(self):
        """
        Initialize an instance of the PoseEstimationNode.
        """
        super().__init__("pose_estimation", allow_undeclared_parameters=True)

        # listening for transforms btwn apriltag and robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # apriltag list
        self.declare_parameter("apriltag_ids", rclpy.Parameter.Type.STRING_ARRAY)
        self.apriltag_list = (
            self.get_parameter("apriltag_ids").get_parameter_value().string_array_value
        )

        # subscribers
        self.d_sub = self.create_subscription(
            AprilTagDetectionArray, "apriltag/detections", self.detection_callback, 10
        )

        # pose publisher
        self.pose_publisher = self.create_publisher(Pose, "pose_estimate", 10)

    def detection_callback(self, detections):
        """
        Callback when an array of AprilTag detections is received. Return
        robot pose within world frame and publish.
        """
        detection: AprilTagDetection
        for detection in detections:
            if detection.id in self.apriltag_list:
                # find initial relationships
                apriltag_wrt_world = self.tf_buffer.lookup_transform(
                    detection.id, "world", Time()
                )
                apriltag_wrt_camera = self.tf_buffer.lookup_transform(
                    detection.id, "camera", Time()
                )
                # find camera_wrt_apriltag
                transform = tf.concatenate_matrices(
                    tf.translation_matrix(apriltag_wrt_camera),
                    tf.quaternion_matrix(rot),
                )
                # inversed_transform = t.inverse_matrix(transform)
                # find camera_wrt_world
                # publish updated pose estimate


def main(args=None):
    rclpy.init(args=args)
    pose_estimation = PoseEstimationNode()
    rclpy.spin(pose_estimation)
    pose_estimation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
