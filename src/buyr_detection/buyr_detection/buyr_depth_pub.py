import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import numpy as np

class YoloToDepthNode(Node):
    def __init__(self):
        super().__init__('yolo_to_depth_node')
        self.bridge = CvBridge()
        self.latest_depth_frame = None
        
        # --- [1. camera_info에서 확인한 실제 내상수 값] ---
        self.fx = 1003.51025390625
        self.fy = 1003.177001953125
        self.cx_offset = 686.8571166992188
        self.cy_offset = 339.330322265625

        # --- [2. 그리퍼 오프셋 설정 (단위: mm)] ---
        # 카메라 렌즈 중심(0,0) 대비 그리퍼 중심의 위치입니다.
        # 예: 그리퍼가 카메라보다 8cm(80mm) 아래에 있다면 -80.0
        self.gripper_x_offset = 0.0   # 좌우 편차
        self.gripper_y_offset = -80.0 # 상하 편차 (8cm 아래)
        
        # 1. Depth 이미지 구독
        self.depth_sub = self.create_subscription(
            Image, '/oak/stereo/image_raw', self.depth_callback, 10)
        
        # 2. YOLO 결과 구독
        self.yolo_sub = self.create_subscription(
            String, '/buyr_YOLO', self.yolo_callback, 10)
            
        # 3. 최종 결과 발행
        self.result_pub = self.create_publisher(String, '/buyr_detection_final', 10)

        self.get_logger().info("🚀 [Alignment Mode] Camera-to-Gripper Node Started!")

    def depth_callback(self, msg):
        # 16UC1 (mm 단위) 변환
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def yolo_callback(self, msg):
        if self.latest_depth_frame is None:
            return

        try:
            detections = json.loads(msg.data)
            if not detections:
                return

            target = detections[0]
            
            # YOLO 원본 좌표 및 해상도
            raw_cx = float(target['cx'])
            raw_cy = float(target['cy'])
            rgb_w = float(target.get('img_w', 1280.0))
            rgb_h = float(target.get('img_h', 720.0))
            
            # 현재 Depth 프레임 해상도 확인
            d_h, d_w = self.latest_depth_frame.shape
            
            # 좌표 스케일링 (1280x720이 아니더라도 대응 가능)
            cx = raw_cx * (d_w / rgb_w)
            cy = raw_cy * (d_h / rgb_h)

            # --- [Depth 추출: High Detail 방식] ---
            size = 3  # 7x7 영역
            roi = self.latest_depth_frame[max(0, int(cy-size)):min(d_h, int(cy+size+1)), 
                                          max(0, int(cx-size)):min(d_w, int(cx+size+1))]
            
            # 23cm 근접 측정을 위한 필터링 (mm 단위)
            valid_depths = roi[(roi > 150) & (roi < 700)]
            
            if valid_depths.size > 0:
                z_mm = float(np.median(valid_depths)) # 물체까지의 거리 (Z)

                # --- [핵심: 3D 역투영 공식] ---
                # 픽셀 좌표를 실제 세계의 mm 좌표로 변환 (카메라 렌즈 중심 기준)
                real_x = (cx - self.cx_offset) * z_mm / self.fx
                real_y = (cy - self.cy_offset) * z_mm / self.fy

                # --- [그리퍼 중심 정렬] ---
                # 최종 목적지 = 물체 좌표 + 카메라-그리퍼 오프셋
                final_x = real_x + self.gripper_x_offset
                final_y = real_y + self.gripper_y_offset
                final_z = z_mm
                
                # 디버깅 로그
                self.get_logger().info(
                    f"🎯 {target['class']} | Depth: {z_mm:.1f}mm | "
                    f"Gripper Target(mm): X={final_x:.1f}, Y={final_y:.1f}"
                )
                
                # 결과 데이터 구성
                target['real_x_mm'] = final_x
                target['real_y_mm'] = final_y
                target['real_z_mm'] = final_z
                
                res_msg = String()
                res_msg.data = json.dumps(target)
                self.result_pub.publish(res_msg)
            else:
                self.get_logger().warn("⚠️ 유효한 Depth 값을 찾을 수 없습니다.")
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloToDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
