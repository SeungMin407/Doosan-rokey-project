from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import torch
from pathlib import Path
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors

bridge = CvBridge()
device = 'cuda' if torch.cuda.is_available() else 'cpu'

# work space
save_dir = os.path.expanduser('~/Desktop/detect_classification/rgb_images')
os.makedirs(save_dir, exist_ok=True)
work_dir = os.path.expanduser('~/Desktop/detect_classification')
os.makedirs(work_dir, exist_ok=True)
os.chdir(work_dir)

model = YOLO('yolov8s.pt')

class CameraListener(Node):
    def __init__(self):
        super().__init__('camera_listener')
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.listener_callback,
            10
        )
        self.count = 0

    def listener_callback(self, msg):
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = model.predict(
            source=cv_image,
            conf=0.25,                
            save=False,          
            device=device
        )
        # save
        if len(results[0].boxes)!=0  :
            file_path = os.path.join(save_dir, f"frame_{self.count:04d}.png")
            cv2.imwrite(file_path, cv_image)
            self.get_logger().info(f"Saved {file_path}")
            print(f"save: {save_dir}")
            self.count += 1
        else:
            print("no save") 



def main(args=None):
    rclpy.init(args=args)
    node = CameraListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("End")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
