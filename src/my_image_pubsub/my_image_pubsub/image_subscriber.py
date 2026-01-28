import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/image', self.cb, 10
        )
        self.get_logger().info("Subscribing to /image (showing with OpenCV)")

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("ROS2 /image", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

