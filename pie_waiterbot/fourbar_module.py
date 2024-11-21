import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String


class Sensor:
    COLOR = "color"
    STRAINGAUGE = "gauge"

    class Colors:
        BLUE = 0
        GREEN = 1
        RED = 2


class Motor:
    WHEELBASE = "base"
    STEPPER = "stepper"

    class Action:
        TRAY = 20
        DRINK = 30


class FourbarModule(Node):

    def __init__(self):
        super().__init__("fourbar_module")
        self.publisher_ = self.create_publisher(String, "fourbar", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # attributes
        receiving_com_port = "/dev/ttyACM0"
        publishing_com_port = "/dev/ttyACM0"
        baud_rate = 9600
        self.receiving_port = serial.Serial(receiving_com_port, baud_rate, timeout=1)
        self.publishing_port = serial.Serial(publishing_com_port, baud_rate, timeout=1)

    def timer_callback(self):
        data = self.receiving_port.readline().decode()
        if len(data) > 0:
            Sensor.COLOR
        return


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
