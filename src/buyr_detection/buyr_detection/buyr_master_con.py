#!/usr/bin/env python3
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def get_key(settings):
    """키보드 입력을 실시간(Non-blocking)으로 감지하는 함수"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class MasterStateMachineNode(Node):

    def __init__(self):
        super().__init__('master_state_machine_node')

        # 1. 상태 및 모드 설정
        self.mode = 'MODE_3'
        self.current_step = 'IDLE'
        self.is_running = False

        # 2. Publisher & Subscriber
        self.step_cmd_pub = self.create_publisher(String, '/step_cmd', 10)
        self.mode_select_sub = self.create_subscription(
            String, '/system_mode', self.mode_select_callback, 10
        )
        self.step_status_sub = self.create_subscription(
            String, '/step_status', self.status_callback, 10
        )
        self.master_cmd_sub = self.create_subscription(
            String, '/master_cmd', self.master_cmd_callback, 10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('👑 [Master State Machine Node] 시스템 준비 완료')
        self.get_logger().info(f' 현재 설정된 모드: {self.mode}')
        self.get_logger().info('=' * 60)

    def print_menu(self):
        print("""
==================================================
🎮 [키보드 직접 제어 가이드]
--------------------------------------------------
 [s] : 시퀀스 시작 (START)
 [q] : 시퀀스 긴급 정지 (STOP)
 [3] : 3단계 모드 선택
 [6] : 6단계 모드 선택
 [Ctrl+C] : 프로그램 종료
==================================================
        """)

    def publish_cmd(self, cmd_str):
        msg = String()
        msg.data = cmd_str
        self.step_cmd_pub.publish(msg)
        self.get_logger().info(f'📤 [마스터 명령 발행]: {cmd_str}')

    def mode_select_callback(self, msg):
        mode_input = msg.data.strip()
        if mode_input in ['3', 'MODE_3', '3STEP']:
            self.mode = 'MODE_3'
            self.get_logger().info('🔄 모드 변경 완료 -> [3단계 모드]')
        elif mode_input in ['6', 'MODE_6', '6STEP']:
            self.mode = 'MODE_6'
            self.get_logger().info('🔄 모드 변경 완료 -> [6단계 모드]')
        else:
            self.get_logger().warn(f'⚠️ 알 수 없는 모드 입력: {mode_input}')

    def master_cmd_callback(self, msg):
        cmd = msg.data.strip().upper()

        if cmd == 'START':
            if self.is_running:
                self.get_logger().warn('⚠️ 시퀀스가 이미 진행 중입니다.')
                return

            self.is_running = True
            self.get_logger().info(
                f'🚀 [시퀀스 시작] 설정된 모드: {self.mode}'
            )

            self.current_step = 'STEP1'
            self.publish_cmd('step1')

        elif cmd == 'STOP':
            self.is_running = False
            self.current_step = 'IDLE'
            self.publish_cmd('STOP')
            self.get_logger().warn('🛑 마스터 긴급 정지 완료.')

    def status_callback(self, msg):
        status = msg.data.strip().lower()
        self.get_logger().info(
            f'📥 [하위 노드 상태 수신]: {status} (현재단계: {self.current_step})'
        )

        if not self.is_running:
            return

        # 3단계 모드
        if self.mode == 'MODE_3':
            if status == 'step1_complete' and self.current_step == 'STEP1':
                self.current_step = 'STEP2'
                time.sleep(0.5)
                self.publish_cmd('step2')

            elif status == 'step2_complete' and self.current_step == 'STEP2':
                self.current_step = 'STEP3'
                time.sleep(0.5)
                self.publish_cmd('step3')

            elif status == 'step3_complete' and self.current_step == 'STEP3':
                self.get_logger().info(
                    '🎉 [3단계 모드 완료] 모든 시퀀스가 정상 완료되었습니다.'
                )
                self.is_running = False
                self.current_step = 'IDLE'

        # 6단계 모드
        elif self.mode == 'MODE_6':
            if status == 'step1_complete' and self.current_step == 'STEP1':
                self.current_step = 'STEP2'
                time.sleep(0.5)
                self.publish_cmd('step2_6')

            elif status == 'step2_complete' and self.current_step == 'STEP2':
                self.current_step = 'STEP3'
                time.sleep(0.5)
                self.publish_cmd('step3_6')

            elif status == 'step3_complete' and self.current_step == 'STEP3':
                self.current_step = 'STEP4'
                time.sleep(0.5)
                self.publish_cmd('step4_6')

            elif status == 'step4_complete' and self.current_step == 'STEP4':
                self.current_step = 'STEP5'
                time.sleep(0.5)
                self.publish_cmd('step5_6')

            elif status == 'step5_complete' and self.current_step == 'STEP5':
                self.current_step = 'STEP6'
                time.sleep(0.5)
                self.publish_cmd('step6_6')

            elif status == 'step6_complete' and self.current_step == 'STEP6':
                self.get_logger().info(
                    '🎉 [6단계 모드 완료] 모든 시퀀스가 정상 완료되었습니다.'
                )
                self.is_running = False
                self.current_step = 'IDLE'


def keyboard_loop(node, settings):
    """키보드 입력을 지속적으로 모니터링하여 내부 콜백/명령을 실행하는 스레드 함수"""
    node.print_menu()
    while rclpy.ok():
        key = get_key(settings)
        if key in ['s', 'S']:
            # 내부 콜백 로직 직접 호출
            msg = String()
            msg.data = 'START'
            node.master_cmd_callback(msg)
        elif key in ['q', 'Q']:
            msg = String()
            msg.data = 'STOP'
            node.master_cmd_callback(msg)
        elif key == '3':
            msg = String()
            msg.data = '3'
            node.mode_select_callback(msg)
        elif key == '6':
            msg = String()
            msg.data = '6'
            node.mode_select_callback(msg)
        elif key == '\x03':  # Ctrl+C
            break


def main(args=None):
    # 터미널 설정을 저장
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = MasterStateMachineNode()

    # 키보드 입력을 처리할 별도 스레드 시작
    input_thread = threading.Thread(
        target=keyboard_loop, args=(node, settings), daemon=True
    )
    input_thread.start()

    try:
        # ROS 2 통신 처리 (스피닝)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 터미널 상태 복원
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
