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
            String, "fourbar_module_angle", self.fourbar_callback, 10
        )
        """self.declare_parameter("serial_write", rclpy.Parameter.Type.STRING)
        serial_write = (
            self.get_parameter("serial_write").get_parameter_value().string_value
        )"""
        serial_write = "HELLO"
        try:
            self.write_port = serial.Serial(serial_write, baudRate, timeout=1)
            self.get_logger().info("Write serial connected")
        except:
            self.write_port = None
            self.get_logger().info(f"Write serial failed to connect")

        # for reading
        """self.read_timer = self.create_timer(0.01, self.read_callback)
        self.declare_parameter("serial_read", rclpy.Parameter.Type.STRING)
        serial_read = (
            self.get_parameter("serial_read").get_parameter_value().string_value
        )"""
        serial_read = "HELLO"
        try:
            self.read_port = serial.Serial(serial_read, baudRate, timeout=1)
            self.get_logger().info("Read serial connected")
        except:
            self.read_port = None
            self.get_logger().info("Read serial failed to connect")

        # publishers
        self.goal_publisher = self.create_publisher(String, "goal_id", 10)
        self.serial_publisher = self.create_publisher(String, "drivetrain_encoder", 10)
        self.serial_publisher = self.create_publisher(String, "red_button", 10)
        self.serial_publisher = self.create_publisher(String, "green_button", 10)
        self.serial_publisher = self.create_publisher(String, "blue_button", 10)
        self.serial_publisher = self.create_publisher(String, "imu", 10)
        self.serial_publisher = self.create_publisher(String, "strain_gauge", 10)
        self.serial_publisher = self.create_publisher(String, "color_sensor", 10)

    def cmd_callback(self, twist: Twist):
        """
        Callback function when a Twist command is received. Transmit it onto
        the serial port for the microcontroller.
        """
        serial_line = self.cfg_msg("DT", f"{twist.linear.x},{twist.angular.z}")
        self.get_logger().info(f"SERIAL PUBLISH: {serial_line}")
        if self.write_port is not None:
            self.write_port.write(serial_line.encode())

    def fourbar_callback(self, string: String):
        """
        Callback function for the fourball_module topic. Gets a String message
        which includes an int signifying the a"ngle that the fourbar should be
        at. Transmit it onto the serial port of the microcontroller.
        """
        serial_line = self.cfg_msg("ST", string.data)
        self.get_logger().info(f"SERIAL PUBLISH: {serial_line}")
        if self.write_port is not None:
            self.write_port.write(serial_line.encode())

    def read_callback(self):
        """
        Decode the latest line of serial sensor data.
        """
        if self.read_port is not None:
            data = self.read_port.readline().decode()
        else:
            data = ""
        if len(data) > 0:
            # goal = String()
            # goal.data = data
            # self.goal_publisher.publish(goal)
            """msg_arr = string.data.split(",")
            if msg_arr[0] == "CL":
                match msg_arr[1]:
                    case "1":
                        self.task_mode = "TRAY"
                    case "2":
                        self.task_mode = "DRINK"
                    case _:
                        self.task_mode = "TRAY"
            if msg_arr[0] == "SG":
                if msg_arr[1] == "false":
                    self.task_status = 2"""

    def cfg_msg(self, code, msg):
        """
        Reconfigure messages sent to the microcontroller in a format that could
        be interepreted by the microcontroller.

        DT,${linear},${angular}
        ST,${stepper motor angle}
        """
        return code + "," + msg


def main(args=None):
    rclpy.init(args=args)
    serial_adapter = SerialAdapterNode()
    rclpy.spin(serial_adapter)
    serial_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
