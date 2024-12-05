import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float32MultiArray
from geometry_msgs.msg import Twist

import serial


class SerialAdapterNode(Node):
    """
    Acts as a translator for incoming (sensor data) and outgoing (motor
    commands) information between the ROS2 node network and the robot firmware.
    """

    def __init__(self):
        """
        Initialize an instance of the SerialAdapterNode class.
        """
        super().__init__("serial_adapter", allow_undeclared_parameters=True)
        baudRate = 9600

        # retrieve outgoing codes
        self.declare_parameter("drivetrain_code", rclpy.Parameter.Type.INTEGER)
        self.dt_code = (
            self.get_parameter("drivetrain_code").get_parameter_value().integer_value
        )
        self.declare_parameter("stepper_code", rclpy.Parameter.Type.INTEGER)
        self.st_code = (
            self.get_parameter("stepper_code").get_parameter_value().integer_value
        )

        # retrieve goals associated w/ buttons and place in dict
        self.button_dest_table = {}
        self.declare_parameter("button_list", rclpy.Parameter.Type.STRING_ARRAY)
        for code in (
            self.get_parameter("button_list").get_parameter_value().string_array_value
        ):
            self.declare_parameter(code, rclpy.Parameter.Type.STRING)
            self.button_dest_table[code] = (
                self.get_parameter(code).get_parameter_value().string_value
            )

        # subscriptions to outgoing data
        self.speeds_subscriber = self.create_subscription(
            Twist, "cmd_vel", self.drivetrain_callback, 10
        )
        self.fourbar_angle = self.create_subscription(
            String, "fourbar_module_angle", self.fourbar_callback, 10
        )

        # publishers for incoming data
        self.goal_request_publisher = self.create_publisher(String, "goal_request", 10)
        self.drivetrain_publisher = self.create_publisher(
            Float32MultiArray, "drivetrain_encoder", 10
        )
        self.imu_publisher = self.create_publisher(Float32MultiArray, "imu", 10)
        self.strain_publisher = self.create_publisher(Bool, "strain_gauge", 10)
        self.color_publisher = self.create_publisher(String, "color_sensor", 10)

        # for writing
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

    def drivetrain_callback(self, twist: Twist):
        """
        Callback function when a Twist message for commanding the drivetrain
        motors is received. Transmit it onto the serial port for the
        microcontroller.
        """
        serial_line = self.cfg_msg(self.dt_code, f"{twist.linear.x},{twist.angular.z}")
        self.get_logger().info(f"SERIAL PUBLISH: {serial_line}")
        if self.write_port is not None:
            self.write_port.write(serial_line.encode())
        else:
            self.get_logger().info(f"Catching serial message: {serial_line}")

    def fourbar_callback(self, string: String):
        """
        Callback function for the fourball_module topic. Gets a String message
        which includes an int signifying the angle that the fourbar should be
        at. Transmit it onto the serial port of the microcontroller.
        """
        serial_line = self.cfg_msg(self.st_code, string.data)
        self.get_logger().info(f"SERIAL PUBLISH: {serial_line}")
        if self.write_port is not None:
            self.write_port.write(serial_line.encode())
        else:
            self.get_logger().info(f"Catching serial message: {serial_line}")

    def read_callback(self):
        """
        Decode the latest line of serial sensor data.
        """
        # decode data or save an empty string
        if self.read_port is not None:
            data = self.read_port.readline().decode()
        else:
            data = ""
        if len(data) > 0:
            # split up message and sort by letter code
            msg_arr = data.split(",")

            # first, handle buttons
            if msg_arr[0] in self.button_dest_table.keys():
                self.goal_request_publisher.publish(self.button_dest_table[msg_arr[0]])
            # encoder data
            elif msg_arr[0] == "EN":
                self.drivetrain_publisher.publish(
                    Float32MultiArray(data=[float(msg_arr[1]), float(msg_arr[2])])
                )
            # IMU data
            elif msg_arr[0] == "MU":
                self.imu_publisher.publish(
                    Float32MultiArray(data=[float(msg_arr[1]), float(msg_arr[2])])
                )
            # strain gauge
            elif msg_arr[0] == "SG":
                if msg_arr[1] == "0":
                    boolean = False
                if msg_arr[1] == "1":
                    boolean = True
                else:
                    return
                self.strain_publisher.publish(Bool(data=boolean))
            # color sensor
            elif msg_arr[0] == "CL":
                self.color_publisher.publish(String(data=msg_arr[1]))

    def cfg_msg(self, code, msg) -> str:
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
