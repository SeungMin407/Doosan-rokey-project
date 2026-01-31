import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion

from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker
import numpy as np
import cv2
import threading
import math
from ultralytics import YOLO

YOLO_MODEL_PATH = '/home/rokey/rokey_ws/src/go_turtle/go_turtle/best.pt'


class DepthToMap(Node):
    def __init__(self):
        super().__init__('depth_to_map')

        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 10)

        self.bridge = CvBridge()
        self.model = YOLO(YOLO_MODEL_PATH)
        self.K = None
        self.lock = threading.Lock()

        ns = self.get_namespace()
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        self.depth_image = None
        self.rgb_image = None
        self.clicked_point = None
        self.shutdown_requested = False
        self.display_image = None

        # --- Car tracking state (per-frame) ---
        self.car_center = None      # (cx, cy) in RGB pixel
        self.car_found = False      # True only if 'car' bbox exists in CURRENT frame

        # GUI thread
        self.gui_thread_stop = threading.Event()
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2
        self.navigator = TurtleBot4Navigator()
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        # logs
        self.logged_intrinsics = False
        self.logged_rgb_shape = False
        self.logged_depth_shape = False

        # subscriptions
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 1)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 1)

        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")
        self.timer = self.create_timer(0.2, self.display_images)
        self.start_timer.cancel()

    def camera_info_callback(self, msg: CameraInfo):
        with self.lock:
            self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            if not self.logged_intrinsics:
                self.get_logger().info(
                    f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, "
                    f"cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}"
                )
                self.logged_intrinsics = True

    def depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth is not None and depth.size > 0:
                if not self.logged_depth_shape:
                    self.get_logger().info(f"Depth image received: {depth.shape}")
                    self.logged_depth_shape = True
                with self.lock:
                    self.depth_image = depth
                    self.camera_frame = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"Depth CV bridge conversion failed: {e}")

    def rgb_callback(self, msg: CompressedImage):
        try:
            # CompressedImage -> BGR
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if rgb is None or rgb.size == 0:
                return

            if not self.logged_rgb_shape:
                self.get_logger().info(f"RGB image decoded: {rgb.shape}")
                self.logged_rgb_shape = True

            # copy latest depth for distance text overlay
            with self.lock:
                depth_mm = self.depth_image.copy() if self.depth_image is not None else None

            draw = rgb.copy()

            # --- YOLO inference ---
            results = self.model.predict(source=rgb, verbose=False)
            r0 = results[0]

            # --- Per-frame car selection (largest bbox) ---
            car_found_local = False
            car_center_local = None
            best_area = 0

            if r0.boxes is not None and len(r0.boxes) > 0:
                boxes = r0.boxes.xyxy.cpu().numpy()
                confs = r0.boxes.conf.cpu().numpy()
                clss  = r0.boxes.cls.cpu().numpy().astype(int)
                names = self.model.names

                for (x1, y1, x2, y2), conf, cls_id in zip(boxes, confs, clss):
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    # distance text (optional)
                    if depth_mm is not None:
                        dist_m = self.get_depth_m(depth_mm, cx, cy)
                        dist_text = f"{dist_m:.2f}m" if np.isfinite(dist_m) else "depth:N/A"
                    else:
                        dist_text = "depth:WAIT"

                    # class name
                    if isinstance(names, dict):
                        cls_name = names.get(cls_id, str(cls_id))
                    else:
                        cls_name = names[cls_id] if 0 <= cls_id < len(names) else str(cls_id)

                    label = f"{cls_name} {conf:.2f} | {dist_text}"

                    # draw bbox + center
                    cv2.rectangle(draw, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(draw, (cx, cy), 3, (0, 0, 255), -1)

                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
                    y_top = max(0, y1 - th - 8)
                    cv2.rectangle(draw, (x1, y_top), (x1 + tw + 6, y1), (0, 255, 0), -1)
                    cv2.putText(draw, label, (x1 + 3, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)

                    if isinstance(cls_name, str) and cls_name.lower() == 'car':
                        area = (x2 - x1) * (y2 - y1)
                        if area > best_area:
                            best_area = area
                            car_center_local = (cx, cy)
                            car_found_local = True

            with self.lock:
                self.rgb_image = rgb
                self.display_image = draw
                self.car_center = car_center_local
                self.car_found = car_found_local

        except Exception as e:
            self.get_logger().error(f"Compressed RGB decode / YOLO failed: {e}")

    def get_depth_m(self, depth_mm: np.ndarray, x: int, y: int) -> float:
        h, w = depth_mm.shape[:2]
        x = int(np.clip(x, 0, w - 1))
        y = int(np.clip(y, 0, h - 1))

        d = float(depth_mm[y, x])
        if d > 0 and np.isfinite(d):
            return d / 1000.0

        r = 3
        patch = depth_mm[max(0, y-r):y+r+1, max(0, x-r):x+r+1].astype(np.float32)
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0]
        if patch.size == 0:
            return float('nan')
        return float(np.median(patch)) / 1000.0

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            with self.lock:
                self.clicked_point = (x, y)
            self.get_logger().info(f"Clicked RGB pixel: ({x}, {y})")

    def display_images(self):

        with self.lock:
            img = self.display_image.copy() if self.display_image is not None else None
            depth = self.depth_image.copy() if self.depth_image is not None else None
            click = self.clicked_point
            frame_id = getattr(self, 'camera_frame', None)

        if img is None or depth is None or frame_id is None:
            return

        try:
            if click is not None:
                x, y = click
                # bounds safety
                h, w = depth.shape[:2]
                x = int(np.clip(x, 0, w - 1))
                y = int(np.clip(y, 0, h - 1))

                z = float(depth[y, x]) / 1000.0
                cv2.circle(img, (x, y), 4, (0, 255, 0), -1)

                if self.K is not None:
                    fx, fy = float(self.K[0, 0]), float(self.K[1, 1])
                    cx, cy = float(self.K[0, 2]), float(self.K[1, 2])
                    X = (x - cx) * z / fx
                    Y = (y - cy) * z / fy
                    Z = z

                    pt_camera = PointStamped()
                    pt_camera.header.stamp = Time().to_msg()
                    pt_camera.header.frame_id = frame_id
                    pt_camera.point.x, pt_camera.point.y, pt_camera.point.z = float(X), float(Y), float(Z)
                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        frame_id,
                        Time(),
                        timeout=Duration(seconds=1.0)
                    )

                    # apply transform
                    pt_map = do_transform_point(pt_camera, transform)

                    self.publish_goal_marker(pt_map.point.x, pt_map.point.y)
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.header.stamp = self.get_clock().now().to_msg()
                    goal_pose.pose.position.x = pt_map.point.x
                    goal_pose.pose.position.y = pt_map.point.y

                    yaw = 0.0
                    goal_pose.pose.orientation = Quaternion(
                        x=0.0, y=0.0,
                        z=math.sin(yaw / 2.0),
                        w=math.cos(yaw / 2.0)
                    )

                    self.navigator.goToPose(goal_pose)

                with self.lock:
                    self.clicked_point = None

            with self.lock:
                self.display_image = img

        except Exception as e:
            self.get_logger().warn(f"TF or goal error: {e}")

    def track_car(self):
        SAFE_DIST = 0.25      # meters (원하는 안전거리)
        MIN_Z_SAFE = 0.2    # 너무 가까우면 목표점이 비정상(0/음수) 되는 걸 방지

        with self.lock:
            img = self.display_image.copy() if self.display_image is not None else None
            depth = self.depth_image.copy() if self.depth_image is not None else None
            frame_id = getattr(self, 'camera_frame', None)
            car_found = bool(getattr(self, 'car_found', False))
            car_center = self.car_center

        if img is None or depth is None or frame_id is None:
            return

        if (not car_found) or (car_center is None):
            self.get_logger().info("track_car: 'car' bbox not found in current RGB frame.")
            return

        try:
            x, y = car_center

            h, w = depth.shape[:2]
            x = int(np.clip(x, 0, w - 1))
            y = int(np.clip(y, 0, h - 1))

            z = float(depth[y, x]) / 1000.0  # car distance (m)

            # visualize tracking point
            cv2.circle(img, (x, y), 6, (255, 0, 0), -1)

            if self.K is not None:
                # prevent_collision
                z_safe = z - SAFE_DIST

                # 너무 가까우면 중단
                if z_safe < MIN_Z_SAFE:
                    self.get_logger().warn(
                        f"track_car: too close (z={z:.2f}m). "
                        f"cannot keep {SAFE_DIST:.2f}m gap. skip."
                    )
                    with self.lock:
                        self.display_image = img
                    return

                fx, fy = float(self.K[0, 0]), float(self.K[1, 1])
                cx, cy = float(self.K[0, 2]), float(self.K[1, 2])

                X = (x - cx) * z_safe / fx
                Y = (y - cy) * z_safe / fy
                Z = z_safe

                pt_camera = PointStamped()
                pt_camera.header.stamp = Time().to_msg()
                pt_camera.header.frame_id = frame_id
                pt_camera.point.x, pt_camera.point.y, pt_camera.point.z = float(X), float(Y), float(Z)

                transform = self.tf_buffer.lookup_transform(
                    'map',
                    frame_id,
                    Time(),
                    timeout=Duration(seconds=1.0)
                )

                pt_map = do_transform_point(pt_camera, transform)

                #self.publish_goal_marker(pt_map.point.x, pt_map.point.y)
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = pt_map.point.x
                goal_pose.pose.position.y = pt_map.point.y

                yaw = 0.0
                goal_pose.pose.orientation = Quaternion(
                    x=0.0, y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0)
                )

                self.get_logger().info(
                    f"track_car: goToPose SAFE map(x={goal_pose.pose.position.x:.2f}, "
                    f"y={goal_pose.pose.position.y:.2f}) | z={z:.2f}m -> z_safe={z_safe:.2f}m"
                )
                self.navigator.goToPose(goal_pose)

            else:
                self.get_logger().info(
                    f"track_car: depth out of range or intrinsics missing (z={z:.2f}m)"
                )

            with self.lock:
                self.display_image = img

        except Exception as e:
            self.get_logger().warn(f"track_car: TF or goal error: {e}")
    def publish_goal_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE # 구체 모양
        marker.action = Marker.ADD

        # 위치 설정
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2 # 바닥보다 조금 높게

        # 크기 설정 (중요: 너무 작으면 안 보임)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # 색상 설정 (빨간색, 투명도 없음)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


    def gui_loop(self):
        cv2.namedWindow('RGB', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RGB', 640, 480)
        cv2.moveWindow('RGB', 100, 100)
        cv2.setMouseCallback('RGB', self.mouse_callback)

        while not self.gui_thread_stop.is_set():
            with self.lock:
                img = self.display_image.copy() if self.display_image is not None else None

            if img is not None:
                cv2.imshow('RGB', img)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    self.get_logger().info("Shutdown requested by user (via GUI).")
                    try:
                        self.navigator.dock()
                    except Exception as e:
                        self.get_logger().warn(f"dock failed: {e}")
                    self.shutdown_requested = True
                    self.gui_thread_stop.set()
                    rclpy.shutdown()

                elif key == ord('a'):
                    self.track_car()
            else:
                cv2.waitKey(10)


def main():
    rclpy.init()
    node = DepthToMap()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.gui_thread_stop.set()
    node.gui_thread.join()
    node.destroy_node()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
