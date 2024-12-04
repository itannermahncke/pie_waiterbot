import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Twist, Transform

from tf_transformations import euler_from_quaternion

import math


class GoalDriverNode(Node):
    """
    Given a pose estimate, compare the robot pose to the known location of
    AprilTags, knowing both are in the world frame. Calculate robot Twist
    and publish to a topic available to the MicroROS node.
    """

    def __init__(self):
        """
        Initialize an instance of the GoalDriverNode class.
        """
        super().__init__("goal_driver", allow_undeclared_parameters=True)

        # apriltag pose management
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # parameters
        self.declare_parameter("apriltag_ids", rclpy.Parameter.Type.STRING_ARRAY)
        self.apriltag_id_list = (
            self.get_parameter("apriltag_ids").get_parameter_value().string_array_value
        )
        self.declare_parameter("initial_pose", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.latest_coords = (
            self.get_parameter("initial_pose").get_parameter_value().double_array_value
        )

        # goal management
        self.goal_subscriber = self.create_subscription(
            String, "goal_id", self.goal_update_callback, 10
        )
        self.goal_status_pub = self.create_publisher(Bool, "goal_status", 10)
        self.goal_status_sub = self.create_subscription(
            Bool, "goal_status", self.goal_status_callback, 10
        )

        # drive management
        self.pose_subscriber = self.create_subscription(
            Pose, "pose_estimate", self.pose_update_callback, 10
        )
        self.speed_interval = self.create_timer(0.1, self.control_loop)
        self.speeds_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # goal attributes
        self.latest_goal_id = None
        self.goal_status = True  # start frozen

        # control
        self.ang_K = 0.1
        self.lin_K = 0.1
        self.max_ang_vel = 0.9436
        self.max_lin_vel = 0.2720
        self.tolerance = 0.1

    def goal_update_callback(self, goal_id: String):
        """
        Callback function when a button press indicates that the robot has a
        new goal AprilTag to navigate towards.
        """
        if goal_id.data in self.apriltag_id_list:
            self.latest_goal_id = goal_id.data
            self.goal_status_pub.publish(Bool(data=False))
        else:
            self.get_logger().info(
                f"ERROR: ID {goal_id.data} NOT FOUND IN KNOWN ID LIST."
            )

    def goal_status_callback(self, status_msg: Bool):
        """
        Update status attribute.
        """
        self.goal_status = status_msg.data

    def pose_update_callback(self, pose: Pose):
        """
        Callback function when a new pose estimate is received. Transforms the
        pose into usable coordinates.
        """
        heading = euler_from_quaternion(
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )[2]
        self.latest_coords = (pose.position.x, pose.position.y, heading)

    def control_loop(self):
        """
        Calculate wheel speeds and publish to a topic accessible to the microcontroller.
        """
        twist = Twist()
        # only do this if goal exists and is not yet met
        if self.latest_goal_id is not None and not self.goal_status:
            # calculate error
            lin_error, ang_error = self.calculate_error()

            # if error is significant, correct
            if lin_error > self.tolerance or ang_error > self.tolerance:
                twist.linear.x = min(lin_error * self.lin_K, self.max_lin_vel)
                twist.angular.z = min(ang_error * self.ang_K, self.max_ang_vel)
            # if within tolerance, stop and change goal state
            else:
                self.goal_status_pub.publish(Bool(data=True))

            # publish
            self.get_logger().info(
                f"publishing lin: {twist.linear.x} ang {twist.angular.z}"
            )
            self.speeds_publisher.publish(twist)

    def calculate_error(self):
        """
        Calculate error between current heading and ideal heading to approach AprilTag.
        """
        goal_xy = self.tf_buffer.lookup_transform(
            "world", self.latest_goal_id, Time()
        ).transform.translation
        delta_x = goal_xy.x - self.latest_coords[0]
        delta_y = goal_xy.y - self.latest_coords[1]
        lin_error = math.sqrt(delta_x**2 + delta_y**2)
        ang_error = math.atan2(delta_y, delta_x) - self.latest_coords[2]

        self.get_logger().info(f"lin_error: {lin_error} | ang_error: {ang_error}")
        return lin_error, ang_error


def main(args=None):
    rclpy.init(args=args)
    goal_driver = GoalDriverNode()
    rclpy.spin(goal_driver)
    goal_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
