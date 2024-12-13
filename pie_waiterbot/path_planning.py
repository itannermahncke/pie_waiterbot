import rclpy
import rclpy.logging
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import String, Bool, Int16
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

        # map parameters
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
            String, "goal_request", self.request_callback, 10
        )
        self.module_status_sub = self.create_subscription(
            Int16, "fourbar_module_status", self.fourbar_status_callback, 10
        )

        # robot initial status parameters
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
        self.declare_parameter("module_status", rclpy.Parameter.Type.INTEGER)
        self.module_status = (
            self.get_parameter("module_status").get_parameter_value().integer_value
        )

        # request management and path planning type: INTERMITTENT or CONTINUAL
        self.declare_parameter("path_mode", rclpy.Parameter.Type.STRING)
        self.path_mode = (
            self.get_parameter("path_mode").get_parameter_value().string_value
        )
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
        if self.goal_status == True:
            self.get_logger().info(f"Completed the latest goal: {self.latest_goal_id}")

    def fourbar_status_callback(self, status_msg: Int16):
        """
        0: not started
        1: extend the module
        2: retracting the module
        3: Just finished
        """
        self.module_status = status_msg.data
        self.get_logger().info(f"Heard latest module update: {self.module_status}")

        if self.module_status == 3:
            # attempt to determine the next goal
            self.determine_next_goal()

    def request_callback(self, request: String):
        """
        Handles a goal change request from a button.
        """
        self.get_logger().info(f"Heard request: {request.data}")
        # only handle verified destinations
        if request.data in self.destinations:
            self.get_logger().info(f"Added {request.data} to destinations!")
            self.request_queue.append(request.data)
            self.determine_next_goal()
        else:
            self.get_logger().info(f"Request {request.data} not in destinations!")

    def determine_next_goal(self):
        """
        Determines whether or not to publish the next goal in the queue.
        """
        # only move forward if the current goal is complete and the module is stationary
        if self.goal_status and self.module_status == 3:
            # if currently at a table
            if self.latest_goal_id != self.kitchen and self.latest_goal_id is not None:
                # if intermittent mode, return to kitchen immediately
                # if in continual mode, return to kitchen if queue is empty
                if self.path_mode == "INTERMITTENT" or (
                    self.path_mode == "CONTINUAL" and len(self.request_queue) == 0
                ):
                    self.request_queue.appendleft(self.kitchen)

            # only move forward if there is a next available goal, otherwise do nothing
            if len(self.request_queue) > 0:
                # pop the next goal
                current_goal = String(data=self.request_queue.popleft())

                # publish new goal
                self.get_logger().info(f"Announcing new task {current_goal.data}")
                self.goal_id_publisher.publish(current_goal)
            else:
                self.get_logger().info(f"No goals: {self.request_queue}")


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
