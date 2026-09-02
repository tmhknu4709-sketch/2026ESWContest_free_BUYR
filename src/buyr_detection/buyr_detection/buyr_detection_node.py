import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import json
import torch  # CUDA 확인용

class YoloToggleDetection(Node):
    def __init__(self):
        super().__init__('buyr_toggle_detection_node')

        # --- ROS 설정 ---
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image, '/oak/rgb/image_raw', self.image_callback, 10)
        self.trigger_subscription = self.create_subscription(
            String, '/detect_now', self.trigger_callback, 10)
        self.class_subscription = self.create_subscription(
            String, '/set_target_class', self.param_msg_callback, 10)

        self.result_publisher = self.create_publisher(String, '/buyr_YOLO', 10)
        self.debug_image_publisher = self.create_publisher(Image, '/buyr_debug_image', 10)

        # --- [오린 나노 최적화] YOLO 모델 로드 및 CUDA 이동 ---
        model_path = '/home/taemin/humble_ws/src/buyr_ct.pt'
        self.model = YOLO(model_path)
        
        # GPU 사용 가능 여부 확인 후 이동
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info("🔥 YOLO Model loaded on GPU (CUDA)")
        else:
            self.get_logger().warn("⚠️ CUDA not available, using CPU")

        # --- 상태 변수 ---
        self.target_class = ''
        self.latest_frame = None
        self.is_ready = False
        self.is_detecting = False  

        self.timer_period = 0.05  # FPS 상향 (오린 나노 성능 고려 20FPS 목표)
        self.timer = self.create_timer(self.timer_period, self.detection_loop)

        self.get_logger().info("✅ Orin Nano Optimized YOLO Node started.")

    def param_msg_callback(self, msg):
        class_name = msg.data.strip()
        if class_name:
            self.target_class = class_name
            self.get_logger().info(f"🎯 Target class updated: '{self.target_class}'")

    def image_callback(self, msg):
        # CV_bridge 변환은 CPU 연산이므로 효율적으로 처리
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.is_ready = True

    def trigger_callback(self, msg):
        self.is_detecting = not self.is_detecting
        status = "STARTED" if self.is_detecting else "STOPPED"
        self.get_logger().info(f"Detection {status}!")

    def detection_loop(self):
        if not self.is_detecting or not self.is_ready or self.latest_frame is None:
            return

        frame = self.latest_frame.copy()
        h_orig, w_orig, _ = frame.shape
        
        # --- [변경] 1. YOLO 추론용으로만 이미지를 180도 뒤집기 ---
        flipped_frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        # --- [변경] 2. 뒤집은 이미지(flipped_frame)로 추론 진행 ---
        results = self.model(flipped_frame, verbose=False, device='cuda', half=True)[0]
        
        detections = []
        best_det = None

        for box in results.boxes:
            # 뒤집힌 이미지 기준의 좌표들
            x1_f, y1_f, x2_f, y2_f = map(float, box.xyxy[0])
            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]
            conf = float(box.conf[0])

            if self.target_class and class_name != self.target_class:
                continue

            # --- [변경] 3. 뒤집힌 좌표를 다시 원래 원본 좌표(정방향)로 변환 ---
            # 180도 회전 시: 원래_x = 가로폭 - 뒤집힌_x
            x1 = w_orig - x2_f
            y1 = h_orig - y2_f
            x2 = w_orig - x1_f
            y2 = h_orig - y1_f

            detections.append({
                'class': class_name, 
                'confidence': conf,
                'cx': round((x1 + x2) / 2.0, 2), 
                'cy': round((y1 + y2) / 2.0, 2),
                'img_w': w_orig,
                'img_h': h_orig,
                'bbox': (x1, y1, x2, y2)
            })

        if detections:
            best_det = max(detections, key=lambda d: d['confidence'])
            result_to_publish = [{
                'class': best_det['class'],
                'confidence': round(best_det['confidence'], 2),
                'cx': best_det['cx'], 
                'cy': best_det['cy'],
                'img_w': best_det['img_w'],
                'img_h': best_det['img_h']
            }]
        else:
            result_to_publish = []

        # 시각화 (원본 프레임 위에 정방향으로 바운딩 박스를 그립니다)
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            color = (0, 255, 0) if det == best_det else (128, 128, 128)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)

        msg_out = String()
        msg_out.data = json.dumps(result_to_publish)
        self.result_publisher.publish(msg_out)

        # 디버그 영상 발행 (원래 카메라 시점 그대로 발행됩니다)
        debug_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.debug_image_publisher.publish(debug_img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloToggleDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
