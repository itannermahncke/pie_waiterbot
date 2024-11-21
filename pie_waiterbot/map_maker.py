import rclpy
import rclpy.logging
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Transform, Quaternion, TransformStamped, Vector3

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations as tf


class MapMakerNode(Node):

    def __init__(self):
        super().__init__("map_maker", allow_undeclared_parameters=True)
        # transformation management
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_dynamic_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter("apriltag_ids", rclpy.Parameter.Type.STRING_ARRAY)
        self.apriltag_list = (
            self.get_parameter("apriltag_ids").get_parameter_value().string_array_value
        )
        self.make_apriltag_transformations()

    def make_apriltag_transformations(self):
        """
        Define all of the static transformations between each AprilTag and the
        world coordinate frame.
        """
        for apriltag_id in self.apriltag_list:
            self.get_logger().info(f"MAKING TRANSFORM FOR {apriltag_id}")
            # declare parameters
            self.declare_parameter(
                f"{apriltag_id}_translation", rclpy.Parameter.Type.DOUBLE_ARRAY
            )
            self.declare_parameter(
                f"{apriltag_id}_rotation", rclpy.Parameter.Type.DOUBLE_ARRAY
            )

            # get translation and rotation
            translation = (
                self.get_parameter(f"{apriltag_id}_translation")
                .get_parameter_value()
                .double_array_value
            )
            rotation = (
                self.get_parameter(f"{apriltag_id}_rotation")
                .get_parameter_value()
                .double_array_value
            )

            # convert to transform
            apriltag_wrt_world = self.make_static_transform(
                apriltag_id, translation, rotation
            )
            self.get_logger().info(f"logging {apriltag_wrt_world} (output)")

            # broadcast
            self.tf_static_broadcaster.sendTransform(apriltag_wrt_world)
            self.get_logger().info("transform sent")

    def make_static_transform(
        self, id, translation: list[float], rotation: list[float]
    ):
        """
        Helper function to create a static transform between a single frame
        and the world frame.
        """
        self.get_logger().info("Making a static transform")
        transform_stamped = TransformStamped()
        header = Header()
        transform = Transform()

        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()

        # make translation
        t_vector = Vector3(x=translation[0], y=translation[1], z=translation[2])
        transform.translation = t_vector

        # make rotation
        r_quat = tf.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        q_vector = Quaternion(x=r_quat[0], y=r_quat[1], z=r_quat[2], w=r_quat[3])
        transform.rotation = q_vector

        transform_stamped.header = header
        transform_stamped.transform = transform
        transform_stamped.child_frame_id = id

        self.get_logger().info(f"logging {transform_stamped}")
        return transform_stamped


def main(args=None):
    rclpy.init(args=args)
    node = MapMakerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
