import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Pose, Point, Quaternion
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations as tf
from tf2_msgs.msg import TFMessage
import numpy as np
import time


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
            AprilTagDetectionArray, "detections", self.detection_callback, 10
        )

        # pose publisher
        self.pose_publisher = self.create_publisher(Pose, "pose_estimate", 10)

    def detection_callback(self, detections):
        """
        Callback when an array of AprilTag detections is received. Return
        robot pose within world frame and publish.
        """
        for detection in detections.detections:
            tag_name = "tag36h11:" + str(detection.id)
            if detection.id in (1, 5):
                # find initial relationships
                apriltag_wrt_world = self.tf_buffer.lookup_transform(
                    f"tag{detection.id}", "world", Time()
                )
                can_lookup = {
                    self.tf_buffer.can_transform(
                        detections.header.frame_id,
                        tag_name,
                        detections.header.stamp,
                    )
                }

                try:

                    if can_lookup:
                        apriltag_wrt_camera = self.tf_buffer.lookup_transform(
                            detections.header.frame_id,
                            tag_name,
                            detections.header.stamp,
                        )

                        # find camera_wrt_world
                        vector_world = np.array(
                            [
                                apriltag_wrt_world.transform.translation.x,
                                apriltag_wrt_world.transform.translation.y,
                                0,
                            ]
                        )
                        quaternion_world = np.array(
                            [
                                apriltag_wrt_world.transform.rotation.x,
                                apriltag_wrt_world.transform.rotation.y,
                                apriltag_wrt_world.transform.rotation.z,
                                apriltag_wrt_world.transform.rotation.w,
                            ]
                        )

                        vector_apriltag = np.array(
                            [
                                -apriltag_wrt_camera.transform.translation.x,
                                -apriltag_wrt_camera.transform.translation.y,
                                0,
                            ]
                        )
                        quaternion_apriltag = np.array(
                            [
                                apriltag_wrt_camera.transform.rotation.x,
                                apriltag_wrt_camera.transform.rotation.y,
                                apriltag_wrt_camera.transform.rotation.z,
                                apriltag_wrt_camera.transform.rotation.w,
                            ]
                        )

                        # get inverse of apriltag to camera matrix
                        cam_to_april_mat = np.matmul(
                            tf.inverse_matrix(tf.quaternion_matrix(quaternion_world)),
                            tf.translation_matrix(vector_apriltag),
                        )

                        april_to_world_mat = np.matmul(
                            tf.inverse_matrix(tf.quaternion_matrix(quaternion_world)),
                            tf.inverse_matrix(tf.translation_matrix(vector_world)),
                        )

                        empty_arr = np.array([0, 0, 0, 1])
                        empty_arr = np.transpose(empty_arr)

                        april_tag_pos = np.matmul(april_to_world_mat, empty_arr)
                        april_to_world = tf.translation_matrix(
                            [april_tag_pos[0], april_tag_pos[1], 0]
                        )

                        cam_to_world_mat = np.matmul(april_to_world, cam_to_april_mat)
                        full_transform = np.matmul(cam_to_world_mat, empty_arr)

                        self.get_logger().info(f"Translation: {full_transform}")

                        cur_pos = Point()
                        cur_pos.x = full_transform[0]
                        cur_pos.y = full_transform[1]
                        cur_pos.z = full_transform[2]
                        quat = tf.quaternion_from_matrix(
                            np.matmul(
                                tf.quaternion_matrix(quaternion_world),
                                tf.quaternion_matrix(quaternion_apriltag),
                            ),
                        )
                        euler = tf.euler_from_quaternion(quat)
                        new_z_angle = euler[2] + 1.5708
                        quat = tf.quaternion_from_euler(euler[0], euler[1], new_z_angle)
                        # self.get_logger().info(
                        #    f"Rotation: {tf.euler_from_quaternion(quat)}"
                        # )
                        cur_quat = Quaternion(
                            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
                        )
                        cur_pose = Pose()
                        cur_pose.position = cur_pos
                        cur_pose.orientation = cur_quat
                        self.pose_publisher.publish(cur_pose)
                except:
                    self.get_logger().info("Error occured!")


def main(args=None):
    rclpy.init(args=args)
    pose_estimation = PoseEstimationNode()
    rclpy.spin(pose_estimation)
    pose_estimation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
