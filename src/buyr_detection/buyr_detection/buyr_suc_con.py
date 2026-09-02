#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time

class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')
        
        # 아두이노 UNO 연결 포트 설정
        self.ser = None
        try:
            self.ser = serial.Serial('/dev/ttyUNO', 115200, timeout=1)
            time.sleep(2)  # 아두이노 리셋 대기시간
            self.get_logger().info("✅ [ArduinoBridgeNode] 아두이노 UNO 시리얼 연결 성공!")
        except Exception as e:
            self.get_logger().error(f"❌ [ArduinoBridgeNode] 시리얼 포트 열기 실패: {e}")

        # PWM 제어용 Subscriber (다른 노드들로부터 PWM 수신)
        self.sub_pwm = self.create_subscription(
            Int32,
            '/arduino_pwm',
            self.pwm_callback,
            10
        )

    def pwm_callback(self, msg: Int32):
        """ /arduino_pwm 토픽 수신 시 실행되는 콜백 """
        pwm_val = max(0, min(255, msg.data))  # 0 ~ 255 범위 클램핑
        self.send_pwm(pwm_val)

    def send_pwm(self, pwm_val):
        """ 아두이노 시리얼 포트로 PWM 값 전송 """
        if self.ser and self.ser.is_open:
            try:
                data_str = f"{pwm_val}\n"
                self.ser.write(data_str.encode('utf-8'))
                self.get_logger().info(f"⚡ [ArduinoBridge] 아두이노로 PWM 전송: {pwm_val}")
            except Exception as e:
                self.get_logger().error(f"❌ 시리얼 전송 중 에러 발생: {e}")

    def destroy_node(self):
        # 종료 시 모터 정지를 위해 PWM 0 인가
        self.send_pwm(0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
