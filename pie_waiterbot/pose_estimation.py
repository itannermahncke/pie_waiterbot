import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Pose
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
import tf_transformations as tf


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
        super().__init__("pose_estimation", allow_undeclared_parameters=True)

        # apriltag transform
        self.declare_parameter("apriltag_ids", None)
        self.apriltag_list = (
            self.get_parameter("apriltag_ids").get_parameter_value().string_array_value
        )
        self.tf_dynamic_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscribers
        self.d_sub = self.create_subscription(
            AprilTagDetectionArray, "apriltag/detections", self.detection_callback, 10
        )
        self.t_sub = self.create_subscription(TFMessage, "tf", self.tf_callback, 10)

        # pose publisher
        self.pose_publisher = self.create_publisher(Pose, "pose_estimate", 10)

    def tf_callback(self, tfmessage: TFMessage):
        # process camera-apriltag relationship
        tf: TransformStamped
        for tf in tfmessage.transforms:
            # if an apriltag_wrt_image tf, update apriltag_wrt_camera tf
            if tf.child_frame_id in self.apriltag_list and tf.header != "camera":
                apriltag_wrt_camera = tf
                apriltag_wrt_camera.header = "camera"
                self.tf_dynamic_broadcaster.sendTransform(apriltag_wrt_camera)
            else:
                print(
                    f"tag {tf.child_frame_id} not found in AprilTag list. ignoring tf"
                )

    def detection_callback(self, detections):
        """
        Callback when an array of AprilTag detections is received.
        """
        detection: AprilTagDetection
        for detection in detections:
            if detection.id in self.apriltag_list:
                # find initial relationships
                apriltag_wrt_world = self.tf_buffer.lookup_transform(
                    detection.id, "world"
                )
                apriltag_wrt_camera = self.tf_buffer.lookup_transform(
                    detection.id, "camera"
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
