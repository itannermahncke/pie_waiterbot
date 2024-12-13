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
        baudRate = 115200

        # retrieve outgoing codes
        self.get_logger().info("RETRIEVING PARAMS")
        self.declare_parameter("drivetrain_code", rclpy.Parameter.Type.STRING)
        self.dt_code = (
            self.get_parameter("drivetrain_code").get_parameter_value().string_value
        )
        self.declare_parameter("stepper_code", rclpy.Parameter.Type.STRING)
        self.st_code = (
            self.get_parameter("stepper_code").get_parameter_value().string_value
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
        self.get_logger().info("SUBS")
        self.speeds_subscriber = self.create_subscription(
            Twist, "cmd_vel", self.drivetrain_callback, 10
        )
        self.fourbar_angle = self.create_subscription(
            String, "fourbar_module_angle", self.fourbar_callback, 10
        )

        # publishers for incoming data
        self.get_logger().info("PUBS")
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

        self.get_logger().info("INIT COMPLETE.")

    def drivetrain_callback(self, twist: Twist):
        """
        Callback function when a Twist message for commanding the drivetrain
        motors is received. Transmit it onto the serial port for the
        microcontroller.
        """
        serial_line = self.cfg_msg(self.dt_code, f"{twist.linear.x},{twist.angular.z}")
        if self.write_port is not None:
            self.write_port.write(serial_line.encode())

    def fourbar_callback(self, string: String):
        """
        Callback function for the fourball_module topic. Gets a String message
        which includes an int signifying the angle that the fourbar should be
        at. Transmit it onto the serial port of the microcontroller.
        """
        serial_line = self.cfg_msg(self.st_code, string.data)
        if self.write_port is not None:
            self.write_port.write(serial_line.encode())
            self.get_logger().info(f"SENT FOURBAR ANGLE: {serial_line}")

    def read_callback(self):
        """
        Decode the latest line of serial sensor data.
        """
        # decode data
        if self.read_port is not None:
            line_data = self.read_port.readline().decode()
            self.get_logger().info(f"parsing data: {line_data}")
        else:
            return

        # at this point, data should be present
        if len(line_data) > 0:
            # split up message and sort by letter code
            msg_code = line_data[0:2]
            msg_data = line_data[2:].split(",")

            # first, handle buttons
            if msg_code in self.button_dest_table.keys():
                self.goal_request_publisher.publish(
                    String(data=self.button_dest_table[msg_code])
                )
                self.get_logger().info(f"read button {msg_code}")
            # encoder data
            elif msg_code == "en":
                self.drivetrain_publisher.publish(
                    Float32MultiArray(data=[float(msg_data[0]), float(msg_data[1])])
                )
            # IMU data
            elif msg_code == "mu":
                self.imu_publisher.publish(
                    Float32MultiArray(data=[float(msg_data[0]), float(msg_data[1])])
                )
            # strain gauge
            elif msg_code == "sg":
                if msg_data[0] == "0":
                    boolean = False
                if msg_data[0] == "1":
                    boolean = True
                else:
                    return
                self.strain_publisher.publish(Bool(data=boolean))
                self.get_logger().info(f"read straingauge {msg_data[0]}")
            # color sensor
            elif msg_code == "cl":
                self.color_publisher.publish(String(data=msg_data[0]))

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
