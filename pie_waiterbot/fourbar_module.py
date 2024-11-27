import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String


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
        self.task_status = 0
        # TRAY or DRINKS
        self.task_mode = "TRAY"

        self.serial_subscriber = self.create_subscription(
            String, "serial", self.serial_callback, 10
        )
        # change the name of this topic
        self.goal_subscriber = self.create_subscription(
            String, "destination", self.serial_callback, 10
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

    def serial_callback(self, string: String):
        """
        Takes data from the serial port and determines what module is currently
        loaded.
        """
        msg_arr = string.data.split(":")
        if msg_arr[0] == "COLOR":
            match msg_arr[1]:
                case "300":
                    self.task_mode = "TRAY"
                case "500":
                    self.task_mode = "DRINK"
                case _:
                    self.task_mode = "TRAY"
        if msg_arr[0] == "BUTTON":  # or strain gauge reading
            self.task_status = 2

    def goal_callback(self, string: String):
        """
        Takes a message from the ${topic name} topic and sets the state of the
        module's task.

        0: not started
        1: extend the module
        2: retracting the module
        """
        if string.data == "MOVING":
            self.task_status = 0
        if string.data == "TARGET_REACHED":
            if self.task_status == 0:
                self.task_status = 1

    def timer_callback(self):
        """
        Publishes data
        """
        if self.task_status == 2:
            self.time = self.time + self.publisher_tick_rate
            if self.time > 5:  # wait for 5 seconds before changing status to 0
                self.task_status = 0
                self.time = 0

        angle = String()
        status = String()

        status.data = self.task_status
        if self.task_status in (0, 2):
            angle.data = 0
        if self.task_status == 1:
            if self.task_mode == "TRAY":
                angle.data = 30  # change this
            if self.task_mode == "DRINK":
                angle.data = 45  # change this
            else:
                angle.data = 0

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
