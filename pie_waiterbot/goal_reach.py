import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time
import time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import String, Bool, Empty
from geometry_msgs.msg import Pose, Twist

from tf_transformations import euler_from_quaternion

import math


class ReachGoalNode(Node):
    """
    This node, given a pose estimate, compares the robot pose to its current
    goal, knowing both are in the world frame. Determines a Twist message that
    best corrects the error between the two and publish it.
    """

    def __init__(self):
        """
        Initialize an instance of the ReachGoalNode class.
        """
        super().__init__("goal_reach", allow_undeclared_parameters=True)

        # apriltag pose management
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # map parameters
        self.declare_parameter("destination_ids", rclpy.Parameter.Type.STRING_ARRAY)
        self.destinations = (
            self.get_parameter("destination_ids")
            .get_parameter_value()
            .string_array_value
        )

        # robot parameters
        self.declare_parameter("initial_pose", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.latest_coords = (
            self.get_parameter("initial_pose").get_parameter_value().double_array_value
        )
        self.declare_parameter("goal_id", rclpy.Parameter.Type.STRING)
        self.latest_goal_id = (
            self.get_parameter("goal_id").get_parameter_value().string_value
        )
        if self.latest_goal_id == "None":
            self.latest_goal_id = None
        self.declare_parameter("goal_status", rclpy.Parameter.Type.BOOL)
        self.goal_status = (
            self.get_parameter("goal_status").get_parameter_value().bool_value
        )

        # goal management
        self.goal_id_subscriber = self.create_subscription(
            String, "latest_goal", self.goal_update_callback, 10
        )
        self.goal_status_pub = self.create_publisher(Bool, "goal_status", 10)

        self.goal_status_sub = self.create_subscription(
            Bool, "goal_status", self.latest_status_callback, 10
        )

        # drive management
        self.pose_subscriber = self.create_subscription(
            Pose, "pose_estimate", self.pose_update_callback, 10
        )
        self.speed_interval = self.create_timer(0.05, self.control_loop)
        self.speeds_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.estop_subscriber = self.create_subscription(
            Empty, "e_stop", self.estop_callback, 10
        )

        # control
        self.ang_K = 0.5
        self.lin_K = 0.8
        self.HI_LOW = [0.5, 0.9, 0.15, 0.272]
        self.max_ang_vel = self.HI_LOW[2]
        self.max_lin_vel = self.HI_LOW[3]
        self.ang_tol = 0.2
        self.lin_tol = 0.1

        # latest Twist
        self.latest_twist = Twist()

    def estop_callback(self, _: Empty):
        """
        Immediately stops the motors and prevents further motor commands. Node
        requires relaunch when this occurs for robot to continue working.
        """
        # kills timer and sends a zero-velocity twist
        self.speeds_publisher.publish(Twist())

    def latest_status_callback(self, goal: Bool):
        """
        Update local latest goal status.
        """
        self.goal_status = goal.data
        self.get_logger().info(f"Changed status to {goal.data}")

    def goal_update_callback(self, goal_id: String):
        """
        Callback function when a button press indicates that the robot has a
        new goal to navigate towards.
        """
        if goal_id.data == "kitchen":
            twist = Twist()
            twist.angular.z = 0.5
            self.speeds_publisher.publish(twist)
            time.sleep(8)
            empty_twist = Twist()
            self.speeds_publisher.publish(empty_twist)
        self.latest_goal_id = goal_id.data
        self.goal_status_pub.publish(Bool(data=False))

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
        Calculate wheel speeds and publish.
        """
        twist = Twist()
        empty = True
        # only do this if goal exists and is not yet met
        if self.latest_goal_id is not None and not self.goal_status:
            # calculate error
            lin_error, ang_error = self.calculate_error()
            ang_error = self.angle_normalize(ang_error)
            self.get_logger().info(f"Ang error: {ang_error}, lin error: {lin_error}")

            # if angle error is significant, correct
            if lin_error < self.lin_tol:
                self.get_logger().info(f"No error!")
                self.goal_status_pub.publish(Bool(data=True))
            else:
                if abs(ang_error) > self.ang_tol:
                    self.get_logger().info(f"Correcting angular")
                    twist.angular.z = self.max_ang_vel * (ang_error / abs(ang_error))
                    # round(self.directionless_min(ang_error * self.ang_K, self.max_ang_vel), 6)
                    empty = False
                # if lin error is significant, correct
                elif lin_error > self.lin_tol:
                    self.get_logger().info(f"Correcting linear")
                    twist.linear.x = round(
                        self.directionless_min(
                            lin_error * self.lin_K, self.max_lin_vel
                        ),
                        6,
                    )
                    empty = False
                # if within tolerance, stop and change goal state
                else:
                    self.get_logger().info(f"No error!")
                    self.goal_status_pub.publish(Bool(data=True))

            # publish OR skip if identical to latest
            if not (
                twist.linear.x == self.latest_twist.linear.x
                and twist.angular.z == self.latest_twist.angular.z
            ):
                self.speeds_publisher.publish(twist)
                self.latest_twist = twist
                # if we just sent a zero command, send it again
                if empty:
                    self.speeds_publisher.publish(twist)

    def calculate_error(self):
        """
        Calculate error between current heading and ideal heading to approach
        destination.
        """
        goal_xy = self.tf_buffer.lookup_transform(
            "world", self.latest_goal_id, Time()
        ).transform.translation

        delta_x = goal_xy.x - self.latest_coords[0]
        delta_y = goal_xy.y - self.latest_coords[1]
        lin_error = math.sqrt(delta_x**2 + delta_y**2)
        ang_error = math.atan2(delta_y, delta_x) - self.latest_coords[2]

        return lin_error, ang_error

    def directionless_min(self, velocity, max_vel):
        """
        Return the minimum value between a velocity and its maximum without
        throwing up when the velocity is negative.
        """
        # we want to grab the number that is closest to 0.0
        if velocity >= 0:
            v = min(velocity, max_vel)
        if velocity < 0:
            v = max(velocity, -1 * max_vel)

        return v

    def angle_normalize(self, angle):
        """
        Reduce the angle, forcing it to fit in the range of -pi to pi.
        """
        # get remainder - put on unit circle
        if abs(angle) > 2 * math.pi:
            angle = angle % 2 * math.pi

        # create negatives
        if angle > math.pi:
            angle = angle - 2 * math.pi

        return round(angle, 3)


def main(args=None):
    rclpy.init(args=args)
    node = ReachGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
