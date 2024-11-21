import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import String
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

        # subscribers
        self.goal_subscriber = self.create_subscription(
            String, "goal_id", self.goal_update_callback, 10
        )
        self.pose_subscriber = self.create_subscription(
            Pose, "pose_estimate", self.pose_update_callback, 10
        )

        # publishers
        self.speed_interval = self.create_timer(0.1, self.publish_vel)
        self.speeds_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # attributes
        self.latest_goal_id = None
        self.latest_coords = (0.0, 0.0, 0.0)  # x, y, theta
        self.ang_K = 0.1
        self.lin_K = 0.1
        self.max_ang_vel = 0.9436
        self.max_lin_vel = 0.2720

    def goal_update_callback(self, goal_id: String):
        """
        Callback function when a button press indicates that the robot has a
        new goal AprilTag to navigate towards.
        """
        if goal_id.data in self.apriltag_id_list:
            self.latest_goal_id = goal_id.data
        else:
            print(f"ERROR: ID {goal_id.data} NOT FOUND IN KNOWN ID LIST.")

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

    def publish_vel(self):
        """
        Calculate wheel speeds and publish to a topic accessible to the microcontroller.
        """
        twist = Twist()
        if self.latest_goal_id is None:
            print("No goal yet")
        else:
            error = self.calculate_error()
            twist.linear = error[0] * min(self.lin_K, self.max_lin_vel)
            twist.angular = min(error[1] * min(self.ang_K, self.max_ang_vel))
            self.speeds_publisher.publish(twist)

    def calculate_error(self):
        """
        Calculate error between current heading and ideal heading to approach AprilTag.
        """
        goal_xy: Transform = self.tf_buffer.lookup_transform(
            self.latest_goal_id, "world", Time()
        )
        delta_x = goal_xy.translation.x - self.latest_coords[0]
        delta_y = goal_xy.translation.z - self.latest_coords[1]
        lin_error = math.sqrt(delta_x**2 + delta_y**2)
        ang_error = math.atan2(delta_y, delta_x) - self.latest_coords[2]
        return (lin_error, ang_error)


def main(args=None):
    rclpy.init(args=args)
    goal_driver = GoalDriverNode()
    rclpy.spin(goal_driver)
    goal_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
