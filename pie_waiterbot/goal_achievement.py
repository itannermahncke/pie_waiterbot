import rclpy
from rclpy.node import Node

from constants import apriltag_id_names, apriltag_poses

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from tf_transformations import euler_from_quaternion

import math


class GoalAchievementNode(Node):
    """
    Given a pose estimate, compare the robot pose to the known location of
    AprilTags, knowing both are in the world frame. Calculate robot Twist
    and publish to a topic available to the MicroROS node.
    """

    def __init__(self):
        """
        Initialize an instance of the GoalAchievementNode class.
        """
        super().__init__("goal_achievement")

        self.speed_interval = self.create_timer(0.1, self.publish_vel)

        # subscribers
        self.goal_subscriber = self.create_subscription(
            String, "goal_id", self.goal_update_callback, 10
        )
        self.pose_subscriber = self.create_subscription(
            Pose, "pose_estimate", self.pose_update_callback, 10
        )

        # publishers
        self.speeds_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # attributes
        self.latest_goal_id = apriltag_id_names["Kitchen"]
        self.latest_coords = (0.0, 0.0, 0.0)  # x, y, theta
        self.ang_K = 0.1
        self.lin_K = 0.1

    def goal_update_callback(self, goal_id: String):
        """
        Callback function when a button press indicates that the robot has a
        new goal AprilTag to navigate towards.
        """
        if goal_id.data in apriltag_id_names.keys():
            self.latest_goal_id = goal_id.data
        else:
            print(f"ERROR: ID {goal_id.data} NOT FOUND IN KNOWN ID LIST.")

    def pose_update_callback(self, pose: Pose):
        """
        Callback function when a new pose estimate is received. Transforms the
        pose into usable coordinates.
        """
        heading = euler_from_quaternion(pose.orientation)[2]
        self.latest_coords = (pose.position.x, pose.position.y, heading)

    def publish_vel(self):
        """
        Calculate wheel speeds and publish to a topic accessible to the microcontroller.
        """
        twist = Twist()
        error = self.calculate_error()
        twist.linear = error[0]
        twist.angular = error[1]
        self.speeds_publisher.publish(twist)

    def calculate_error(self):
        """
        Calculate error between current heading and ideal heading to approach AprilTag.
        """
        goal_xy = apriltag_poses
        delta_x = goal_xy[0] - self.latest_coords[0]
        delta_y = goal_xy[1] - self.latest_coords[1]
        lin_error = math.sqrt(delta_x**2 + delta_y**2)
        ang_error = math.atan2(delta_y, delta_x) - self.latest_coords[2]
        return (lin_error, ang_error)


def main(args=None):
    rclpy.init(args=args)
    goal_achievement = GoalAchievementNode()
    rclpy.spin(goal_achievement)
    goal_achievement.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
