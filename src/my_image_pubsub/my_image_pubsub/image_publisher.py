import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.pub = self.create_publisher(Image, '/image', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)  # /dev/video0
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera (VideoCapture(0)).")
            raise RuntimeError("Camera open failed")

        self.timer = self.create_timer(0.1, self.tick)  # 10Hz
        self.get_logger().info("Publishing camera frames to /image")

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Camera frame read failed.")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

