import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class FourbarModule(Node):
    """
    Receives messages from microcontroller and decides what to do with the
    fourbar module.
    """

    def __init__(self):
        super().__init__("fourbar_module")

        # state controls
        # 0: task not started
        # 1: extending module
        # 2: retracting module
        # 3: just finished
        self.task_status = 1
        # TRAY or DRINKS
        self.task_mode = "TRAY"
        self.destination = None

        # subscribe to sensor topics
        self.color_subscriber = self.create_subscription(
            String, "color_sensor", self.color_callback, 10
        )
        self.strain_gauge_subsciber = self.create_subscription(
            String, "strain_gauge", self.strain_gauge_callback, 10
        )

        # change the name of this topic
        self.goal_subscriber = self.create_subscription(
            Bool, "goal_status", self.goal_callback, 10
        )
        self.location_subscriber = self.create_subscription(
            String, "goal_id", self.location_callback, 10
        )

        self.angle_publisher = self.create_publisher(String, "fourbar_module_angle", 10)
        self.status_publisher = self.create_publisher(
            String, "fourbar_module_status", 10
        )
        self.publisher_tick_rate = 0.1
        self.publish_timer = self.create_timer(
            self.publisher_tick_rate, self.timer_callback
        )
        self.time = 0

    def color_callback(self, string: String):
        """
        Takes data from the color_sensor topic and determines what module is currently
        loaded.
        """
        if string == "1":
            self.task_mode = "TRAY"
        elif string == "2":
            self.task_mode = "DRINK"

    def strain_gauge_callback(self, string: String):
        """
        Takes data from the strain_gauge topic and changes the task status
        """
        if self.task_status == 1:
            if string == "false":
                self.task_status = 2
        print("HI")

    def goal_callback(self, boolean: Bool):
        """
        Takes a boolean from the goal_status topic and sets the state of the
        module's task.

        0: not started
        1: extend the module
        2: retracting the module
        3: Just finished
        """
        if boolean.data:
            if self.task_status in (0, 3):
                self.task_status = 1

    def location_callback(self, goal_id: String):
        """
        Takes an int specifying target location
        """
        if not goal_id.data == self.destination:
            self.task_status = 0
        self.destination = goal_id.data

    def timer_callback(self):
        """
        Publishes data
        """
        if self.task_status == 2:
            self.time = self.time + self.publisher_tick_rate
            if self.time > 5:  # wait for 5 seconds before changing status to 0
                self.task_status = 3
                self.time = 0

        angle = String()
        status = String()

        cur_angle = 0

        status.data = str(self.task_status)
        if self.task_status in (0, 2, 3):
            cur_angle = 0
        if self.task_status == 1:
            if self.task_mode == "TRAY":
                cur_angle = 30  # change this
            elif self.task_mode == "DRINK":
                cur_angle = 45  # change this
            else:
                cur_angle = 0

        angle.data = str(cur_angle)
        # self.get_logger().info(f"angle: {cur_angle}, status: {self.task_status}")

        self.status_publisher.publish(status)
        self.angle_publisher.publish(angle)


def main(args=None):
    rclpy.init(args=args)

    fourbar_module = FourbarModule()

    rclpy.spin(fourbar_module)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fourbar_module.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
