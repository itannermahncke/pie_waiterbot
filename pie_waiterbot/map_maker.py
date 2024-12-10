import rclpy
import rclpy.logging
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import (
    Transform,
    TransformStamped,
    Quaternion,
    TransformStamped,
    Vector3,
)
from tf2_msgs.msg import TFMessage

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations as tf


class MapMakerNode(Node):
    """
    This node manages static and dynamic transformations of objects in relation
    to each other and the world frame. It centrally focuses on the relationship
    between the robot and apriltags, as well as between the apriltags and the world.
    """

    def __init__(self):
        super().__init__("map_maker", allow_undeclared_parameters=True)

        # transformation management
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_dynamic_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.t_sub = self.create_subscription(TFMessage, "tf", self.tf_callback, 10)

        # apriltag placement
        self.declare_parameter("apriltag_ids", rclpy.Parameter.Type.STRING_ARRAY)
        self.apriltag_list = (
            self.get_parameter("apriltag_ids").get_parameter_value().string_array_value
        )
        at_tfs = self.make_apriltag_transformations()

        # destination placement
        self.declare_parameter("destination_ids", rclpy.Parameter.Type.STRING_ARRAY)
        self.dest_list = (
            self.get_parameter("destination_ids")
            .get_parameter_value()
            .string_array_value
        )
        dt_tfs = self.make_dest_transformations()

        self.get_logger().info(f"AT TRANSFORMS: {at_tfs}")

        self.get_logger().info(f"DT TRANSFORMS: {dt_tfs}")

        self.tf_static_broadcaster.sendTransform(at_tfs + dt_tfs)

    def make_apriltag_transformations(self):
        """
        Define all of the static transformations between each AprilTag and the
        world coordinate frame.
        """
        tf_list = []
        for apriltag_id in self.apriltag_list:
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

            # broadcast
            tf_list.append(apriltag_wrt_world)
        return tf_list

    def make_dest_transformations(self):
        """
        Define all of the static transformations between each destination and
        the world coordinate frame.
        """
        tf_list = []
        for dest in self.dest_list:
            # declare parameters
            self.declare_parameter(f"{dest}_coords", rclpy.Parameter.Type.DOUBLE_ARRAY)

            # get translation and rotation
            coords = (
                self.get_parameter(f"{dest}_coords")
                .get_parameter_value()
                .double_array_value
            )

            # convert to transform
            dest_wrt_world = self.make_static_transform(dest, coords, [0.0, 0.0, 0.0])

            # broadcast
            tf_list.append(dest_wrt_world)
        return tf_list

    def make_static_transform(
        self, id, translation: list[float], rotation: list[float]
    ):
        """
        Helper function to create a static transform between a single frame
        and the world frame.
        """
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

        self.get_logger().info(
            f"Made transform {transform_stamped.header} {transform_stamped.child_frame_id}"
        )
        return transform_stamped

    def tf_callback(self, tfmessage: TFMessage):
        """
        When a new tf message describes the relationship between an apriltag
        and the camera plane, send it to the transform manager.
        """
        # process camera-apriltag relationship
        tf: TransformStamped
        for tf in tfmessage.transforms:
            print(tf)
            # if an apriltag_wrt_image tf, update apriltag_wrt_camera tf
            if tf.child_frame_id in ["tag36h11:1"] and tf.header.frame_id != "camera":
                apriltag_wrt_camera = tf
                apriltag_wrt_camera.header.frame_id = "camera"
                self.tf_dynamic_broadcaster.sendTransform(apriltag_wrt_camera)


def main(args=None):
    rclpy.init(args=args)
    node = MapMakerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
