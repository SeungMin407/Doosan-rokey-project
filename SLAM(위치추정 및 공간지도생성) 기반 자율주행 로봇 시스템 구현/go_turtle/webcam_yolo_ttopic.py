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
        self.camera_index = 2
        # =================

        self.pub_det = self.create_publisher(String, '/yolo/detections', 10)
        self.pub_has = self.create_publisher(Bool, '/yolo/has_detection', 10)
        self.pub_which = self.create_publisher(String, '/yolo/which_target', 10)

        self.model = YOLO("best.pt")
        self.get_logger().info(f"Model classes: {self.model.names}")

        # 초기 publish
        init_msg = Bool()
        init_msg.data = False
        self.pub_has.publish(init_msg)

        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Webcam open failed")
        else:
            self.get_logger().info("✅ Webcam opened")

        self.hit_count = 0
        self.armed = False   # ✅ publish 기준

    def run(self):
        self.get_logger().info("🚀 run loop started")

        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                break

            results = self.model(frame, verbose=False)
            r0 = results[0]

            target_found = False
            target_det_list = []

            # ===============================
            # YOLO 결과 처리 + 박스 수집
            # ===============================
            if r0.boxes is not None:
                for i in range(len(r0.boxes)):
                    conf = float(r0.boxes.conf[i])
                    cls = int(r0.boxes.cls[i])
                    name = r0.names.get(cls, str(cls))

                    # 🔴 디버그용: 모든 박스는 노란색으로 그림
                    x1, y1, x2, y2 = map(int, r0.boxes.xyxy[i].tolist())
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 1)
                    cv2.putText(
                        frame,
                        f'{name} {conf:.2f}',
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        1
                    )

                    # 🔵 실제 타겟 필터
                    if conf < self.conf_thres:
                        continue
                    if name not in self.target_names:
                        continue

                    target_det_list.append({
                        "xyxy": [x1, y1, x2, y2],
                        "conf": conf,
                        "cls": cls,
                        "name": name
                    })
                    target_found = True

                    # 🔵 필터 통과 박스는 빨간색
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

            # ===============================
            # hit / stable / armed 로직
            # ===============================
            if target_found:
                self.hit_count += 1
            else:
                self.hit_count = 0

            stable_true = (self.hit_count >= self.hit_needed)

            # ✅ stable은 판단만, armed는 상태
            if stable_true:
                self.armed = True

            # ===============================
            # publish (armed만)
            # ===============================
            has_msg = Bool()
            has_msg.data = self.armed
            self.pub_has.publish(has_msg)

            if stable_true:
                det_msg = String()
                det_msg.data = json.dumps({
                    "stable": True,
                    "armed": self.armed,
                    "hit_count": self.hit_count,
                    "num": len(target_det_list),
                    "detections": target_det_list
                })
                self.pub_det.publish(det_msg)

                best = max(target_det_list, key=lambda d: d["conf"])
                which_msg = String()
                which_msg.data = best["name"]
                self.pub_which.publish(which_msg)

            # ===============================
            # 화면 표시
            # ===============================
            cv2.putText(
                frame,
                f"armed={self.armed} hit={self.hit_count} stable={stable_true}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
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
