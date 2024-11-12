import rclpy
from rclpy.node import Node
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
        super().__init__("serial_adapter")

        # publishers
        self.speeds_subscriber = self.create_subscription(
            Twist, "cmd_vel", self.cmd_callback, 10
        )

        # attributes
        arduinoComPort = "/dev/ttyACM0"
        baudRate = 9600
        self.port = serial.Serial(arduinoComPort, baudRate, timeout=1)

    def cmd_callback(self, twist: Twist):
        """
        Callback function when a Twist command is received. Transmit it onto
        the serial port for the microcontroller.
        """
        serial_line = (f"{twist.linear}", {twist.angular})
        self.port.write(serial_line)


def main(args=None):
    rclpy.init(args=args)
    serial_adapter = SerialAdapterNode()
    rclpy.spin(serial_adapter)
    serial_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
