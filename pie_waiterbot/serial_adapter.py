import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import serial


class SerialAdapterNode(Node):
    """
    Receives Twist messages and transmits them serially to a microcontroller.
    """

    def __init__(self):
        """
        Initialize an instance of the GoalDriverNode class.
        """
        super().__init__("serial_adapter", allow_undeclared_parameters=True)

        baudRate = 9600

        # for writing
        self.speeds_subscriber = self.create_subscription(
            Twist, "cmd_vel", self.cmd_callback, 10
        )
        self.declare_parameter("serial_write", rclpy.Parameter.Type.STRING)
        serial_write = (
            self.get_parameter("serial_write").get_parameter_value().string_value
        )
        self.write_port = serial.Serial(serial_write, baudRate, timeout=1)

        # for reading
        self.read_timer = self.create_timer(0.01, self.read_callback)
        self.declare_parameter("serial_read", rclpy.Parameter.Type.STRING)
        serial_read = (
            self.get_parameter("serial_read").get_parameter_value().string_value
        )
        # self.read_port = serial.Serial(serial_read, baudRate, timeout=1)

        # publishers
        self.goal_publisher = self.create_publisher(String, "goal_id", 10)

    def cmd_callback(self, twist: Twist):
        """
        Callback function when a Twist command is received. Transmit it onto
        the serial port for the microcontroller.
        """
        serial_line = (f"{twist.linear.x}", {twist.angular.z})
        self.write_port.write(serial_line)

    def read_callback(self):
        """
        Decode the latest line of serial sensor data.
        """
        # data = self.read_port.readline().decode()
        data = ""
        if len(data) > 0:
            goal = String()
            goal.data = data
            self.goal_publisher.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    serial_adapter = SerialAdapterNode()
    rclpy.spin(serial_adapter)
    serial_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
