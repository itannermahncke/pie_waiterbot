import rclpy
from rclpy.node import Node

import math

from geometry_msgs.msg import Pose, Twist
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class DeadReckoningNode(Node):
    """
    Estimate pose with dead reckoning.
    """

    def __init__(self):
        """
        Instantiate an instance of the DeadReckoningNode.
        """
        super().__init__("dead_reckoning")

        self.timestep = 1

        self.pose_timer = self.create_timer(self.timestep, self.spoof_pub)
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_callback, 10)
        self.pose_pub = self.create_publisher(Pose, "pose_estimate", 10)

        self.latest_twist = Twist()  # start with no velocity
        self.latest_pose = Pose()  # start at origin

    def spoof_pub(self):
        """
        Update the robot's immediate pose estimate using dead reckoning principles.
        """
        # setup pose
        new_pose = Pose()
        # assume constant velocity for transformation
        angle_change = self.latest_twist.angular.z * self.timestep
        linear_change = self.latest_twist.linear.x * self.timestep

        # apply change in angle first
        quaternion = []
        quaternion.append(self.latest_pose.orientation.x)
        quaternion.append(self.latest_pose.orientation.y)
        quaternion.append(self.latest_pose.orientation.z)
        quaternion.append(self.latest_pose.orientation.w)
        euler_angles = euler_from_quaternion(quaternion)
        euler_angles[2] == euler_angles[2] + angle_change
        new_angle = quaternion_from_euler(
            euler_angles[0], euler_angles[1], euler_angles[2]
        )
        new_pose.orientation.x = new_angle[0]
        new_pose.orientation.y = new_angle[1]
        new_pose.orientation.z = new_angle[2]
        new_pose.orientation.w = new_angle[3]

        # apply xy change
        angle = 90.0 - angle_change
        new_pose.position.x = (
            linear_change * math.cos(angle) + self.latest_pose.position.x
        )
        new_pose.position.y = (
            linear_change * math.sin(angle) + self.latest_pose.position.y
        )

        # broadcast pose update
        self.latest_pose = new_pose
        self.pose_pub.publish(new_pose)

    def cmd_callback(self, msg: Twist):
        """
        Save the latest Twist command.
        """
        self.latest_twist = msg


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
