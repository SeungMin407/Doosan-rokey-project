import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Bool

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'processed_image',
            self.listener_callback,
            10)
        
        self.detection_subscription = self.create_subscription(
            Bool,
            'is_detected',
            self.detection_callback,
            10)
        
        self.bridge = CvBridge()
        self.last_frame = None
        self.is_detected = False

    def listener_callback(self, msg):
        self.last_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def detection_callback(self, msg):
        # Bool 메시지의 데이터를 클래스 변수에 저장
        self.is_detected = msg.data
        if self.is_detected:
            self.get_logger().info("TRUE : Object Detected!")
        else:
            self.get_logger().info("FALSE : No Object!")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.last_frame is not None:
                cv2.imshow("Processed Image", node.last_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("🛑 'q' pressed, exiting...")
                    break

    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("✅ Subscriber shutdown complete.")

if __name__ == '__main__':
    main()
