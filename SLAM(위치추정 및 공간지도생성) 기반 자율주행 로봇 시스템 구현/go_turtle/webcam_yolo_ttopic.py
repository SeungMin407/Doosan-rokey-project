import cv2
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ultralytics import YOLO


class YoloWebcamNode(Node):
    def __init__(self):
        super().__init__('yolo_webcam_node')

        # ===== 설정 =====
        self.target_names = {"car", "dummy"}
        self.conf_thres = 0.5

        self.hit_needed = 3
        self.miss_needed = 8      # ⭐ 추가: 몇 프레임 연속 못 보면 armed OFF할지

        self.camera_index = 2
        # =================

        self.pub_det = self.create_publisher(String, '/yolo/detections', 10)
        self.pub_has = self.create_publisher(Bool, '/yolo/has_detection', 10)
        self.pub_which = self.create_publisher(String, '/yolo/which_target', 10)

        self.model = YOLO("best.pt")
        self.get_logger().info(f"Model classes: {self.model.names}")

        # ✅ 토픽을 강제로 생성 (노드 뜨자마자) → armed 기준으로 보냄
        init_msg = Bool()
        init_msg.data = False
        self.pub_has.publish(init_msg)
        self.get_logger().info("📢 initial /yolo/has_detection = False")

        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Webcam open failed (토픽은 유지됨)")
        else:
            self.get_logger().info("✅ Webcam opened")

        self.hit_count = 0
        self.miss_count = 0      # ⭐ 추가
        self.armed = False       # ✅ publish는 얘로만

    def run(self):
        self.get_logger().info("🚀 run loop started")

        while rclpy.ok():
            if not self.cap.isOpened():
                # 카메라 없어도 False는 계속 보냄 (armed 기준)
                self.armed = False
                has_msg = Bool()
                has_msg.data = self.armed
                self.pub_has.publish(has_msg)

                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("❌ frame read failed")
                break

            results = self.model(frame, verbose=False)
            r0 = results[0]

            target_found = False
            target_det_list = []

            if r0.boxes is not None:
                for i in range(len(r0.boxes)):
                    conf = float(r0.boxes.conf[i])
                    if conf < self.conf_thres:
                        continue

                    cls = int(r0.boxes.cls[i])
                    name = r0.names.get(cls, str(cls))
                    if name not in self.target_names:
                        continue

                    xyxy = r0.boxes.xyxy[i].tolist()
                    target_det_list.append({
                        "xyxy": xyxy,
                        "conf": conf,
                        "cls": cls,
                        "name": name
                    })
                    target_found = True

            # ✅ hit/miss 로직 (stable 판단 + armed 유지/해제)
            if target_found:
                self.hit_count += 1
                self.miss_count = 0
            else:
                self.hit_count = 0
                self.miss_count += 1

            stable_true = (self.hit_count >= self.hit_needed)           # 🔍 판단용
            stable_false = (self.miss_count >= self.miss_needed)        # 🔍 판단용(OFF 조건)

            # ✅ armed 상태 업데이트 (publish는 armed만)
            if stable_true:
                self.armed = True
            elif stable_false:
                self.armed = False
            # else: 중간 구간에서는 self.armed 유지

            # ✅✅ /yolo/has_detection은 armed만 보냄
            has_msg = Bool()
            has_msg.data = self.armed
            self.pub_has.publish(has_msg)

            # detections / which_target은 "안정 감지(stable_true)" 때만 발행 (원래 의도 유지)
            if stable_true:
                det_msg = String()
                det_msg.data = json.dumps({
                    "stable": True,
                    "armed": self.armed,
                    "hit_count": self.hit_count,
                    "miss_count": self.miss_count,
                    "num": len(target_det_list),
                    "detections": target_det_list
                })
                self.pub_det.publish(det_msg)

                best = max(target_det_list, key=lambda d: d["conf"])
                which_msg = String()
                which_msg.data = best["name"]
                self.pub_which.publish(which_msg)

            # 시각화
            cv2.putText(
                frame,
                f"armed={self.armed} hit={self.hit_count}/{self.hit_needed} miss={self.miss_count}/{self.miss_needed}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if self.armed else (0, 0, 255),
                2
            )
            cv2.imshow("YOLO Webcam", frame)

            rclpy.spin_once(self, timeout_sec=0.0)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = YoloWebcamNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
