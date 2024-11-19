import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from apriltag_msgs.msg import AprilTagDetectionArray
import cv2
import numpy as np


class WebcamDriver(Node):
    def __init__(self):
        super().__init__("webcam_driver")
        self.br = CvBridge()
        self.current_frame = None
        self.x = []
        self.y = []
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
            if len(self.x) != 0:
                pts = np.array(
                    [
                        [self.x[0], self.y[0]],
                        [self.x[1], self.y[1]],
                        [self.x[2], self.y[2]],
                        [self.x[3], self.y[3]],
                    ],
                    np.int32,
                )

                pts = pts.reshape((-1, 1, 2))

                self.current_frame = cv2.polylines(
                    self.current_frame, [pts], True, [255, 0, 0], 2
                )

            if self.i > 2000:
                self.output_video.release()
                print("saving finished")
            else:
                self.output_video.write(self.current_frame)
                self.i = self.i + 1
                print("Saving video...")

    def show_detections(self, detection_array):
        for i in detection_array.detections:
            self.get_logger().info(str(i.corners[0].x))
            self.x = [
                int(i.corners[0].x),
                int(i.corners[1].x),
                int(i.corners[2].x),
                int(i.corners[3].x),
            ]
            self.y = [
                int(i.corners[0].y),
                int(i.corners[1].y),
                int(i.corners[2].y),
                int(i.corners[3].y),
            ]
        if len(detection_array.detections) == 0:
            self.x = []
            self.y = []


def main(args=None):
    rclpy.init(args=args)
    webcam_driver = WebcamDriver()
    rclpy.spin(webcam_driver)
    webcam_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
