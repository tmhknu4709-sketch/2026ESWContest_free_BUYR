#!/usr/bin/env python3
import sys
import select
import tty
import termios
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        
        # 1. 토픽 퍼블리셔 설정
        self.step_pub = self.create_publisher(String, '/step_cmd', 10)
        self.motor_pos_pub = self.create_publisher(JointState, '/motor_cmd_pos', 10)
        
        # 2. 6번 Pitch(Tilt) 모터 제어 변수
        self.pitch_pos = 2050.0
        self.step_size = 50.0
        
        # 터미널 원본 설정 백업
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("==========================================")
        self.get_logger().info("🎹 Step & 6번 Motor Pitch Teleop Node Started")
        self.get_logger().info(" [1] Key : Step 1 실행 (Align / 비전 정렬)")
        self.get_logger().info(" [2] Key : Step 2 실행 (Down & Grip / 하강 및 파지)")
        self.get_logger().info(" [3] Key : Step 3 실행 (Rotate & Discharge / 회전 및 배출)")
        self.get_logger().info(" ----------------------------------------")
        self.get_logger().info(" [4] Key : 6번 Motor Pitch 감소 (-50)")
        self.get_logger().info(" [5] Key : 6번 Motor Pitch 증가 (+50)")
        self.get_logger().info(" [6] Key : 6번 Motor Pitch 초기화 (2050)")
        self.get_logger().info(" [Ctrl+C] : 종료")
        self.get_logger().info("==========================================")

    def get_key(self):
        """매 루프마다 속성을 변경하지 않고 비차단으로 입력만 확인"""
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:
            return sys.stdin.read(1)
        return ''

    def send_pitch_command(self, target_pos):
        msg = JointState()
        msg.name = ['motor_6']
        msg.position = [float(target_pos)]
        self.motor_pos_pub.publish(msg)
        self.get_logger().info(f"📤 [PUB] 6번 Pitch 위치 명령 전송: {target_pos:.1f}")

    def run(self):
        # 실행 시작 시 터미널을 Raw 모드로 변환 (1회 설정)
        tty.setraw(sys.stdin.fileno())
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key:
                    # --- Step 명령 (1, 2, 3) ---
                    if key in ['1', '2', '3']:
                        step_msg = String()
                        step_msg.data = f'step{key}'
                        self.step_pub.publish(step_msg)
                        self.get_logger().info(f"\r\n📤 [PUB] Step {key} 명령 전송 ('step{key}')")
                    
                    # --- 6번 Motor Pitch 각도 제어 (4, 5, 6) ---
                    elif key == '4':
                        self.pitch_pos -= self.step_size
                        self.send_pitch_command(self.pitch_pos)

                    elif key == '5':
                        self.pitch_pos += self.step_size
                        self.send_pitch_command(self.pitch_pos)

                    elif key == '6':
                        self.pitch_pos = 2050.0
                        self.send_pitch_command(self.pitch_pos)

                    elif key == '\x03':  # Ctrl+C
                        break

                time.sleep(0.02)  # 입력 감지 주기 (50Hz)

        except Exception as e:
            self.get_logger().error(f"오류 발생: {e}")
            
        finally:
            # 종료 시 원래 터미널 설정으로 복원
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
