import rclpy
import rclpy.logging
from rclpy.node import Node

import math

from geometry_msgs.msg import Pose, Twist, Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class DeadReckoningNode(Node):
    """
    Estimate pose with dead reckoning. This will be phased out when visual pose
    estimation is working, or combined with it via sensor fusion.
    """

    def __init__(self):
        """
        Instantiate an instance of the DeadReckoningNode.
        """
        super().__init__("dead_reckoning")

        # ros comms stuff
        self.timestep = 0.1
        self.pose_timer = self.create_timer(self.timestep, self.update_pose)
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_callback, 10)
        self.pose_pub = self.create_publisher(Pose, "pose_estimate", 10)

        self.latest_twist = None  # start with no velocity

        # initial pose
        self.declare_parameter("initial_pose", rclpy.Parameter.Type.DOUBLE_ARRAY)
        latest_coords = (
            self.get_parameter("initial_pose").get_parameter_value().double_array_value
        )
        self.latest_pose = Pose()  # start at origin, facing +y axis
        self.latest_pose.position.x = latest_coords[0]
        self.latest_pose.position.y = latest_coords[1]
        self.latest_pose.orientation = self.make_quaternion_msg(
            quaternion_from_euler(0.0, 0.0, latest_coords[2])
        )
        self.pose_pub.publish(self.latest_pose)

    def update_pose(self):
        """
        Update the robot's immediate pose estimate using dead reckoning principles.
        """
        # if no twists have been received, hang out, otherwise estimate pose
        if self.latest_twist is not None:
            # setup new pose
            new_pose = Pose()
            current_pose = self.latest_pose  # avoid interruptions

            # assume constant velocity for transformation
            angle_change = self.latest_twist.angular.z * self.timestep
            linear_change = self.latest_twist.linear.x * self.timestep

            # apply change in angle first
            orient = current_pose.orientation
            euler_angles = list(
                euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            )
            new_heading = self.angle_normalize(euler_angles[2] + angle_change)
            self.get_logger().info(f"ROBOT ANGLE: {euler_angles[2]}")
            new_pose.orientation = self.make_quaternion_msg(
                quaternion_from_euler(euler_angles[0], euler_angles[1], new_heading)
            )

            # apply xy change
            if 0 <= euler_angles[2] < math.pi / 2:
                alpha = euler_angles[2]
                x_dir = 1
                y_dir = 1
            elif math.pi / 2 <= euler_angles[2] < math.pi:
                alpha = math.pi - euler_angles[2]
                x_dir = -1
                y_dir = 1
            elif math.pi <= euler_angles[2] < 3 * math.pi / 2:
                alpha = euler_angles[2] - math.pi
                x_dir = -1
                y_dir = -1
            elif 3 * math.pi / 2 < euler_angles[2] < 2 * math.pi:
                alpha = 2 * math.pi - euler_angles[2]
                x_dir = 1
                y_dir = -1

            # modify xy
            new_pose.position.x = (
                x_dir * linear_change * round(math.cos(alpha), 3)
                + current_pose.position.x
            )
            new_pose.position.y = (
                y_dir * linear_change * round(math.sin(alpha), 3)
                + current_pose.position.y
            )

            # update pose
            self.latest_pose = new_pose

        # broadcast current pose
        self.pose_pub.publish(self.latest_pose)

    def cmd_callback(self, msg: Twist):
        """
        Save the latest Twist command.
        """
        self.latest_twist = msg

    def make_quaternion_msg(self, quat: list[float]):
        """
        Quickly construct a Quaternion message class.
        """
        quaternion = Quaternion()
        quaternion.x = quat[0]
        quaternion.y = quat[1]
        quaternion.z = quat[2]
        quaternion.w = quat[3]

        return quaternion

    def angle_normalize(self, angle):
        """
        Reduce the angle, force it to fit in the range of 0 to 2pi.
        """
        # get remainder - put on unit circle
        if angle > 2 * math.pi:
            angle = angle % 2 * math.pi
        elif angle < 0:
            angle = angle + 2 * math.pi

        return round(angle, 3)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
