#!/usr/bin/env python3
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String


class StepControllerNode(Node):

  def __init__(self):
    super().__init__('step_controller_node')
    self.publisher_ = self.create_publisher(String, '/step_cmd', 10)

    # ★ 브릿지 노드 토픽 이름(/arduino_pwm)에 맞춰 수정
    self.pwm_publisher_ = self.create_publisher(Int32, '/arduino_pwm', 10)

    # 6번 모터 제어를 위한 JointState 퍼블리셔
    self.motor_pos_publisher_ = self.create_publisher(
        JointState, '/motor_cmd_pos', 10
    )

    self.running = True
    self.print_menu()

  def print_menu(self):
    print('\n' + '=' * 60)
    print('    🎮 [스텝 원격 제어 퍼블리셔 노드 (Step 1~6)]')
    print('=' * 60)
    print('    - [1] 키 : Step 1 실행 (정회전)')
    print('    - [2] 키 : Step 2 실행 (YOLO 정렬)')
    print('    - [3] 키 : Step 3 실행 (역회전 원상 복구)')
    print('    - [4] 키 : Step 4 실행 (LiDAR 정렬 및 38cm 접근)')
    print('    - [5] 키 : Step 5 실행 (PWM 128 + M4 자동 왕복)')
    print('    - [6] 키 : Step 6 실행 (배출 동작 시퀀스)')
    print('    - [M] 키 : 6번 모터 제어 (속도: 100, 위치: 600)')
    print('    - [P] 키 : PWM 0 강제 차단 (안전장치)')
    print('    - [A] 키 : 전체 시퀀스 연속 실행 (Step 1 -> 2 -> 4)')
    print('    - [S] 키 : 비상 정지 (STOP)')
    print('    - [Ctrl+C] : 종료')
    print('=' * 60 + '\n')

  def send_cmd(self, cmd_str):
    msg = String()
    msg.data = cmd_str
    self.publisher_.publish(msg)
    self.get_logger().info(f"📤 [토픽 발행] /step_cmd -> '{cmd_str}'")

  def send_pwm_cmd(self, pwm_val):
    """아두이노 브릿지 노드로 direct PWM 명령 발행"""
    msg = Int32()
    msg.data = int(pwm_val)
    self.pwm_publisher_.publish(msg)
    self.get_logger().warn(
        f'🛡️ [안전장치 발행] /arduino_pwm -> {pwm_val}'
    )

  def send_motor6_pos_cmd(self, target_pos=600, target_vel=100):
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = ['motor_6']
    msg.position = [float(target_pos)]
    msg.velocity = [float(target_vel)]

    self.motor_pos_publisher_.publish(msg)
    self.get_logger().info(
        f'⚙️ [모터 제어 발행] /motor_cmd_pos -> ID: 6, Pos: {target_pos}, Vel:'
        f' {target_vel}'
    )


def key_loop(node, settings):
  while node.running and rclpy.ok():
    try:
      tty.setraw(sys.stdin.fileno())
      rlist, _, _ = select.select([sys.stdin], [], [], 0.05)

      if rlist:
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        if key == '1':
          node.send_cmd('STEP_1')
        elif key == '2':
          node.send_cmd('STEP_2')
        elif key == '3':
          node.send_cmd('STEP_3')
        elif key == '4':
          node.send_cmd('STEP_4')
        elif key == '5':
          node.send_cmd('STEP_5')
        elif key == '6':
          node.send_cmd('STEP_6')
        elif key in ['m', 'M']:
          node.send_motor6_pos_cmd(target_pos=600, target_vel=100)
        elif key in ['p', 'P']:
          node.send_pwm_cmd(0)
        elif key in ['a', 'A']:
          node.send_cmd('AUTO')
        elif key in ['s', 'S']:
          node.send_cmd('STOP')
        elif key == '\x03':  # Ctrl+C
          node.running = False
          break
      else:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    except Exception:
      termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
      break


def main(args=None):
  rclpy.init(args=args)
  node = StepControllerNode()
  settings = termios.tcgetattr(sys.stdin)

  input_thread = threading.Thread(target=key_loop, args=(node, settings))
  input_thread.daemon = True
  input_thread.start()

  try:
    while node.running and rclpy.ok():
      rclpy.spin_once(node, timeout_sec=0.01)
  except Exception as e:
    node.get_logger().error(f'오류 발생: {e}')
  finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
