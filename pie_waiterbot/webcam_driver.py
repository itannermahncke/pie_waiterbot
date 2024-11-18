import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from apriltag_msgs.msg import AprilTagDetectionArray
import cv2


class WebcamDriver(Node):
    def __init__(self):
        super().__init__("webcam_driver")
        self.br = CvBridge()
        self.current_frame = None
        self.x = None
        self.y = None
        self.image_subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.listener_callback,
            10,
        )
        self.apriltag_subscription = self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self.show_detections,
            10,
        )
        self.output_video = cv2.VideoWriter(
            "output.avi", cv2.VideoWriter_fourcc(*"MJPG"), 20.0, (640, 480)
        )
        self.i = 0

    def listener_callback(self, img_data):
        try:
            self.current_frame = self.br.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8"
            )
        except CvBridgeError as e:
            self.get_logger().error(e)

        if self.current_frame is not None:
            if self.x is not None:
                self.current_frame = cv2.circle(
                    self.current_frame,
                    (self.x, self.y),
                    radius=10,
                    color=(0, 0, 255),
                    thickness=-1,
                )

            if self.i > 500:
                self.output_video.release()
                print("saving finished")
            else:
                self.output_video.write(self.current_frame)
                self.i = self.i + 1
                print("Saving video...")

    def show_detections(self, detection_array):
        for i in detection_array.detections:
            self.get_logger().info(str(i.centre.x))
            self.x = int(i.centre.x)
            self.y = int(i.centre.y)
        if len(detection_array.detections) == 0:
            self.x = None
            self.y = None


def main(args=None):
    rclpy.init(args=args)
    webcam_driver = WebcamDriver()
    rclpy.spin(webcam_driver)
    webcam_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
