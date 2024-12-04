import tty
import select
import sys
import termios
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class TeleopSerialNode(Node):
    """
    Node that allows teleoperated control of the Waiterbot. This is meant to be
    run independently of the rest of the system for the purposes of testing
    the drivetrain motors.
    """

    def __init__(self):
        """
        Initialize an instance of the TeleopSerialNode.
        """
        # initialize from base Node class
        super().__init__("teleop_serial")

        # key settings
        self.settings = termios.tcgetattr(sys.stdin)

        # publish twist commands to Neato
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # timer for key checks
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # standard velocities
        self.lin = 0.45
        self.ang = 0.4

        # attributes
        arduinoComPort = "/dev/ttyACM0"
        baudRate = 9600
        self.port = serial.Serial(arduinoComPort, baudRate, timeout=1)

    def timer_callback(self):
        """
        Callback for timer; checks key inputs and uses them to publish Twist
        commands.
        """
        # start constructing message
        twist_msg = Twist()

        # get key
        current_key = self.get_key()

        # build message by key
        match current_key:
            case "i":
                twist_msg.linear.x = self.lin
            case "k":
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
            case ",":
                twist_msg.linear.x = self.lin * -1
            case "j":
                twist_msg.angular.z = self.ang
            case "l":
                twist_msg.angular.z = self.ang * -1
            case "u":
                twist_msg.linear.x = self.lin
                twist_msg.angular.z = self.ang
            case "o":
                twist_msg.linear.x = self.lin
                twist_msg.angular.z = self.ang * -1
            case "m":
                twist_msg.linear.x = self.lin * -1
                twist_msg.angular.z = self.ang * -1
            case ".":
                twist_msg.linear.x = self.lin * -1
                twist_msg.angular.z = self.ang

        # publish Twist
        self.cmd_publisher.publish(twist_msg)
        serial_line = str.encode(f"{twist_msg.linear.x}, {twist_msg.angular.z}")
        self.port.write(serial_line)

    def get_key(self):
        """
        Get key input.
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


def main(args=None):
    """
    Main function for rclpy.
    """
    rclpy.init(args=args)
    node = TeleopSerialNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
