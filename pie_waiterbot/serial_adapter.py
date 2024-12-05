import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32MultiArray
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
        self.declare_parameter("serial_write", rclpy.Parameter.Type.STRING)
        serial_write = (
            self.get_parameter("serial_write").get_parameter_value().string_value
        )
        try:
            self.write_port = serial.Serial(serial_write, baudRate, timeout=1)
            self.get_logger().info("Write serial connected")
        except:
            self.write_port = None
            self.get_logger().info("Write serial failed to connect")

        # for reading
        self.read_timer = self.create_timer(0.01, self.read_callback)
        self.declare_parameter("serial_read", rclpy.Parameter.Type.STRING)
        serial_read = (
            self.get_parameter("serial_read").get_parameter_value().string_value
        )
        try:
            self.read_port = serial.Serial(serial_read, baudRate, timeout=1)
            self.get_logger().info("Read serial connected")
        except:
            self.read_port = None
            self.get_logger().info("Read serial failed to connect")

        # publishers
        prefix = "sensor"
        self.goal_publisher = self.create_publisher(String, "goal_id", 10)
        self.drivetrain_publisher = self.create_publisher(
            Float32MultiArray, f"{prefix}/drivetrain_encoder", 10
        )
        self.red_publisher = self.create_publisher(Bool, f"{prefix}/red_button", 10)
        self.green_publisher = self.create_publisher(Bool, f"{prefix}/green_button", 10)
        self.blue_publisher = self.create_publisher(Bool, f"{prefix}/blue_button", 10)
        self.imu_publisher = self.create_publisher(
            Float32MultiArray, f"{prefix}/imu", 10
        )
        self.strain_publisher = self.create_publisher(
            Bool, f"{prefix}/strain_gauge", 10
        )
        self.color_publisher = self.create_publisher(
            String, f"{prefix}/color_sensor", 10
        )

    def cmd_callback(self, twist: Twist):
        """
        Callback function when a Twist command is received. Transmit it onto
        the serial port for the microcontroller.
        """
        serial_line = self.cfg_msg("0", f"{twist.linear.x},{twist.angular.z}")
        self.get_logger().info(f"SERIAL PUBLISH: {serial_line}")
        if self.write_port is not None:
            self.write_port.write(serial_line.encode())

    def fourbar_callback(self, string: String):
        """
        Callback function for the fourball_module topic. Gets a String message
        which includes an int signifying the a"ngle that the fourbar should be
        at. Transmit it onto the serial port of the microcontroller.
        """
        serial_line = self.cfg_msg("1", string.data)
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
            msg_arr = data.data.split(",")
            if msg_arr[0] == "EN":
                self.drivetrain_publisher.publish(
                    Float32MultiArray(data=[float(msg_arr[1]), float(msg_arr[2])])
                )
            elif msg_arr[0] == "BR":
                boolean = False
                if msg_arr[1] == "1":
                    boolean = True
                self.red_publisher.publish(Bool(data=boolean))
            elif msg_arr[0] == "BG":
                if msg_arr[1] == "0":
                    boolean = False
                if msg_arr[1] == "1":
                    boolean = True
                else:
                    return
                self.green_publisher.publish(Bool(data=boolean))
            elif msg_arr[0] == "BB":
                if msg_arr[1] == "0":
                    boolean = False
                if msg_arr[1] == "1":
                    boolean = True
                else:
                    return
                self.blue_publisher.publish(Bool(data=boolean))
            elif msg_arr[0] == "MU":
                self.imu_publisher.publish(
                    Float32MultiArray(data=[float(msg_arr[1]), float(msg_arr[2])])
                )
            elif msg_arr[0] == "SG":
                if msg_arr[1] == "0":
                    boolean = False
                if msg_arr[1] == "1":
                    boolean = True
                else:
                    return
                self.strain_publisher.publish(Bool(data=boolean))
            elif msg_arr[0] == "CL":
                self.color_publisher.publish(String(data=msg_arr[1]))

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
