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
        self.fourbar_angle = self.create_subscription(
            String, "fourbar_module_angle", self.cmd_callback, 10
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
        self.read_port = serial.Serial(serial_read, baudRate, timeout=1)

        # publishers
        self.goal_publisher = self.create_publisher(String, "goal_id", 10)

    def cmd_callback(self, twist: Twist):
        """
        Callback function when a Twist command is received. Transmit it onto
        the serial port for the microcontroller.
        """
        serial_line = f"{twist.linear}, {twist.angular}"
        self.write_port.write(self.cfg_msg(serial_line))

    def fourbar_callback(self, string: String):
        """
        Callback function for the fourball_module topic. Gets a String message
        which includes an int signifying the angle that the fourbar should be
        at. Transmit it onto the serial port of the microcontroller.
        """
        self.write_port.write(self.cfg_msg(string.data))

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

    def cfg_msg(self, msg):
        """
        Reconfigure messages sent to the microcontroller in a format that could
        be interepreted by the microcontroller.

        BASE_MOVEMENT: ${linear}, ${angular}
        FOURBAR_MODULE: ${stepper motor angle}
        """
        val = msg.split(",")
        if len(val) == 1:
            return "FOURBAR_MODULE:" + msg
        if len(val) == 2:
            return "BASE_MOVEMENT:" + msg
        else:
            return "BASE_MOVEMENT:0, 0"


def main(args=None):
    rclpy.init(args=args)
    serial_adapter = SerialAdapterNode()
    rclpy.spin(serial_adapter)
    serial_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
