import rclpy
import rclpy.logging
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import String, Bool
from collections import deque


class PathPlanningNode(Node):
    """
    This node handles the many different ways to trigger a goal change and
    maintains control of the robot's next goal. It does not allow essential
    actions such as module movement to be interrupted, but holds on to
    requests until they can be fulfilled.
    """

    def __init__(self):
        """
        Initialize an instance of the PathPlanningNode class.
        """
        super().__init__("path_planning", allow_undeclared_parameters=True)

        # apriltag pose management
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # parameters
        self.declare_parameter("destination_ids", rclpy.Parameter.Type.STRING_ARRAY)
        self.destinations = (
            self.get_parameter("destination_ids")
            .get_parameter_value()
            .string_array_value
        )
        self.kitchen = self.destinations[0]

        # goal management
        self.goal_id_publisher = self.create_publisher(String, "latest_goal", 10)
        self.goal_id_sub = self.create_subscription(
            String, "latest_goal", self.latest_goal_callback, 10
        )

        # sources of goal change
        self.goal_status_sub = self.create_subscription(
            Bool, "goal_status", self.goal_status_callback, 10
        )
        self.request_sub = self.create_subscription(
            String, "goal_request", self.goal_status_callback, 10
        )
        self.tray_status_sub = self.create_subscription(
            String, "fourbar_module_status", self.fourbar_status_callback, 10
        )

        # attributes
        self.latest_goal_id = None
        self.goal_status = True  # start frozen
        self.module_status = 0  # assume fine
        self.request_queue = deque()

    def latest_goal_callback(self, goal: String):
        """
        Update local latest goal.
        """
        self.latest_goal_id = goal.data

    def goal_status_callback(self, status_msg: Bool):
        """
        Update status attribute.
        """
        # save latest goal
        self.goal_status = status_msg.data

        # if true, determine next destination
        if self.goal_status:
            # always prioritize a return to the kitchen
            if self.latest_goal_id != self.kitchen:
                self.request_queue.appendleft(self.kitchen)
            # else, grab the oldest goal request
            current_goal = self.request_queue.popleft()

            # wait for the robot to be in an acceptable state to start a new goal
            while self.module_status in (1, 2) or not self.goal_status:
                self.get_logger().info("Must complete current task first")

            # publish new goal
            self.goal_id_publisher.publish(current_goal)

    def fourbar_status_callback(self, status_msg: String):
        """
        0: not started
        1: extend the module
        2: retracting the module
        3: Just finished
        """
        self.module_status = status_msg.data

    def request_callback(self, request: String):
        """
        Handles a goal change request from a button.
        """
        # only handle verified destinations
        if request.data in self.destinations:
            self.request_queue.append(request.data)
        else:
            self.get_logger().info(f"Request {request.data} not in destinations!")


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
