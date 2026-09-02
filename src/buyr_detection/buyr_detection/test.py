#!/usr/bin/env python3
import sys
import select
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class FastMultiMotorKeyboardNode(Node):
    def __init__(self):
        super().__init__('fast_multi_motor_keyboard_node')

        # 1. 퍼블리셔 구성 (3, 4번 속도 제어 / 5, 6, 7번 위치 제어)
        self.vel_pub_ = self.create_publisher(JointState, 'motor_cmd_vel', 10)
        self.pos_pub_ = self.create_publisher(JointState, 'motor_cmd_pos', 10)

        # 2. 서브스크라이버 구성 (5번 모터 엔코더 위치 수신 및 원점 잡기)
        self.joint_state_sub = self.create_subscription(
            JointState,
            'motor_encoder_positions',
            self.joint_state_callback,
            10
        )

        # 공용 제어 변수 (3번, 4번 모터용)
        self.current_speed_setting = 100
        self.running = True

        # 3번, 4번 모터 목표 속도 변수
        self.target_speed_3 = 0
        self.target_speed_4 = 0

        self.MIN_SPEED = 0
        self.MAX_SPEED = 1023
        self.STEP = 50

        # ==========================================
        # 5번 모터 위치 제어 변수 (엔코더 기반 원점 적용)
        # ==========================================
        self.m5_base_pos = None      # 수신받은 기준 원점 위치 (Pulse)
        self.m5_current_pos = None   # 실시간 실제 위치 (Pulse)
        self.m5_target_pos = 0       # 목표 위치 (Pulse)
        self.m5_speed = 100          # Profile Velocity (이동 속도)
        self.m5_pos_step = 100       # 이동 단위

        # ==========================================
        # 6번 모터(EX-106) 위치/각도 제어 변수
        # ==========================================
        self.m6_target_pos = 2048    # 초기 목표 위치 (기본 중앙값: 2048)
        self.m6_speed = 50           # 기본 속도 50
        self.m6_pos_step = 50        # 이동 단위

        # ==========================================
        # 7번 모터(Gripper) 위치 제어 변수
        # ==========================================
        self.GRIPPER_OPEN_POS = 2    # 열림 위치
        self.GRIPPER_CLOSE_POS = 2500 # 닫힘 위치
        self.m7_target_pos = 2       # 초기 목표 위치 (열림)

        # 0.05초(20Hz) 주기로 퍼블리시
        self.timer = self.create_timer(0.05, self.publish_cmd)

        self.print_instructions()

    def joint_state_callback(self, msg):
        """ 엔코더 토픽 수신 시 5번 모터의 현재 위치 업데이트 및 기준 원점 지정 """
        if 'motor_5' in msg.name:
            idx = msg.name.index('motor_5')
            self.m5_current_pos = int(msg.position[idx])

            # 노드 실행 후 최초 1회만 현재 위치를 기준 원점(Base)으로 설정
            if self.m5_base_pos is None:
                self.m5_base_pos = self.m5_current_pos
                self.m5_target_pos = self.m5_current_pos
                self.get_logger().info(f"📍 [M5] 기준 원점 자동 설정 완료: {self.m5_base_pos} Pulse")

    def reset_m5_zero_point(self):
        """ 현재 실제 위치를 새로운 원점으로 갱신 """
        if self.m5_current_pos is not None:
            self.m5_base_pos = self.m5_current_pos
            self.m5_target_pos = self.m5_current_pos
            self.update_status_line()
            sys.stdout.write(f"\n📍 [M5] 원점 갱신 완료! 새로운 원점: {self.m5_base_pos} Pulse\n")
            sys.stdout.flush()

    def print_instructions(self):
        print("\n" + "="*75)
        print(" 🚀 3,4번(속도) & 5,6,7번(위치) 통합 키보드 제어 노드")
        print("="*75)
        print(" [3번 모터]   : [E] CW (시계)        | [W] CCW (반시계)")
        print(" [4번 모터]   : [A] CCW (반시계)    | [D] CW (시계)")
        print(" [3,4 속도]   : [U] +50             | [J] -50")
        print("---------------------------------------------------------------------------")
        print(" [5번 미세위치]: [I] 아래로 (+step) | [K] 위로 (-step)")
        print(" [5번 단축키] : [1] 원위치 (Base)   | [0] 목표위치 (Base + 3500)")
        print(" [5번 원점설정]: [Z] 또는 [C] (현재 위치를 새로운 원점으로 갱신) 🔥")
        print(" [5번 단위변경]: [M] 이동단위 증가  | [N] 이동단위 감소")
        print(" [5번 속도]   : [O] 속도 +20        | [L] 속도 -20")
        print("---------------------------------------------------------------------------")
        print(" [6번 EX-106] : [T] 각도 +step      | [G] 각도 -step")
        print(" [6번 단축키] : [R] 중앙 (2048)     | [V] 목표위치 (600, 속도50)")
        print(" [6번 속도]   : [Y] 속도 +20        | [H] 속도 -20")
        print("---------------------------------------------------------------------------")
        print(" [7번 그리퍼] : [[] 열기 (2)         | []] 닫기 (2500)")
        print(" [7번 미세조정]: [P] +100 pulse     | [;] -100 pulse")
        print("---------------------------------------------------------------------------")
        print(" 🛑 [전체 정지]: [SPACE] 또는 [X]    | [Ctrl+C]: 종료")
        print("="*75)
        self.update_status_line()

    def update_status_line(self):
        """한 줄로 상태를 실시간 갱신"""
        m5_str = f"{self.m5_target_pos}p" if self.m5_base_pos is not None else "원점대기.."
        
        sys.stdout.write(
            f"\r⚙️[M3/4]: {self.target_speed_3:<4}/{self.target_speed_4:<4} | "
            f"📍[M5]: {m5_str}(Base:{self.m5_base_pos}, v:{self.m5_speed}) | "
            f"🔄[M6]: {self.m6_target_pos:<5}p(v:{self.m6_speed}) | "
            f"✊[M7]: {self.m7_target_pos:<4}p    "
        )
        sys.stdout.flush()

    def publish_cmd(self):
        # 1. 3번, 4번 모터 속도 제어 퍼블리시
        vel_msg = JointState()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.name = ['motor_3', 'motor_4']
        vel_msg.velocity = [
            float(self.target_speed_3),
            float(self.target_speed_4)
        ]
        self.vel_pub_.publish(vel_msg)

        # 2. 5번, 6번, 7번 모터 위치 제어 퍼블리시
        pos_msg = JointState()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.name = ['motor_5', 'motor_6', 'motor_7']
        pos_msg.position = [
            float(self.m5_target_pos),
            float(self.m6_target_pos),
            float(self.m7_target_pos)
        ]
        pos_msg.velocity = [
            float(self.m5_speed),
            float(self.m6_speed),
            0.0  # 7번 그리퍼 속도는 기본값 사용
        ]
        self.pos_pub_.publish(pos_msg)

    def change_speed(self, amount):
        new_speed = self.current_speed_setting + amount
        self.current_speed_setting = max(self.MIN_SPEED, min(new_speed, self.MAX_SPEED))
        self.update_status_line()

    def change_m5_speed(self, amount):
        new_speed = self.m5_speed + amount
        self.m5_speed = max(0, min(new_speed, 1023))
        self.update_status_line()

    def change_m6_speed(self, amount):
        new_speed = self.m6_speed + amount
        self.m6_speed = max(0, min(new_speed, 1023))
        self.update_status_line()

    def stop_all_motors(self):
        """속도 제어 모터(3, 4번) 정지"""
        self.target_speed_3 = 0
        self.target_speed_4 = 0
        self.update_status_line()


def key_loop(node, settings):
    """키 입력을 비동기로 즉시 처리하는 전용 쓰레드"""
    while node.running and rclpy.ok():
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

            # ==========================================
            # 3번 모터 제어 (W / E)
            # ==========================================
            if key in ['e', 'E']:
                node.target_speed_3 = node.current_speed_setting
                node.update_status_line()
            elif key in ['w', 'W']:
                node.target_speed_3 = -node.current_speed_setting
                node.update_status_line()

            # ==========================================
            # 4번 모터 제어 (A / D)
            # ==========================================
            elif key in ['a', 'A']:
                node.target_speed_4 = node.current_speed_setting
                node.update_status_line()
            elif key in ['d', 'D']:
                node.target_speed_4 = -node.current_speed_setting
                node.update_status_line()

            # 3번, 4번 속도 조절 (U / J)
            elif key in ['u', 'U']:
                node.change_speed(node.STEP)
            elif key in ['j', 'J']:
                node.change_speed(-node.STEP)

            # ==========================================
            # 5번 모터 위치 제어 (I / K / 1 / 0 / Z / C)
            # ==========================================
            elif key in ['i', 'I']:
                if node.m5_base_pos is not None:
                    node.m5_target_pos += node.m5_pos_step
                    node.update_status_line()
            elif key in ['k', 'K']:
                if node.m5_base_pos is not None:
                    node.m5_target_pos -= node.m5_pos_step
                    node.update_status_line()
            elif key == '1':
                # 자동 지정된 원점(Base)으로 이동
                if node.m5_base_pos is not None:
                    node.m5_target_pos = node.m5_base_pos
                    node.update_status_line()
            elif key == '0':
                # 원점 + 3,500 Pulse 위치로 이동
                if node.m5_base_pos is not None:
                    node.m5_target_pos = node.m5_base_pos + 3850
                    node.update_status_line()
            elif key in ['z', 'Z', 'c', 'C']:
                # 🔥 현재 실제 엔코더 위치를 새로운 원점(Base)으로 지정
                node.reset_m5_zero_point()
            elif key in ['m', 'M']:
                node.m5_pos_step = min(1000, node.m5_pos_step + 10)
                node.update_status_line()
            elif key in ['n', 'N']:
                node.m5_pos_step = max(1, node.m5_pos_step - 10)
                node.update_status_line()
            elif key in ['o', 'O']:
                node.change_m5_speed(20)
            elif key in ['l', 'L']:
                node.change_m5_speed(-20)

            # ==========================================
            # 6번 모터 EX-106 각도 제어 (T / G / R / V / Y / H)
            # ==========================================
            elif key in ['t', 'T']:
                node.m6_target_pos += node.m6_pos_step
                node.update_status_line()
            elif key in ['g', 'G']:
                node.m6_target_pos = max(0, node.m6_target_pos - node.m6_pos_step)
                node.update_status_line()
            elif key in ['r', 'R']:
                node.m6_target_pos = 2048  # 중앙값으로 복귀
                node.update_status_line()
            elif key in ['v', 'V']:
                node.m6_target_pos = 600   # 위치 600으로 이동
                node.m6_speed = 50         # 속도 50으로 지정
                node.update_status_line()
            elif key in ['y', 'Y']:
                node.change_m6_speed(20)
            elif key in ['h', 'H']:
                node.change_m6_speed(-20)

            # ==========================================
            # 7번 그리퍼 모터 제어 ([ / ] / P / ;)
            # ==========================================
            elif key == '[':
                node.m7_target_pos = node.GRIPPER_OPEN_POS
                node.update_status_line()
            elif key == ']':
                node.m7_target_pos = node.GRIPPER_CLOSE_POS
                node.update_status_line()
            elif key in ['p', 'P']:
                node.m7_target_pos += 100
                node.update_status_line()
            elif key == ';':
                node.m7_target_pos = max(0, node.m7_target_pos - 100)
                node.update_status_line()

            # ==========================================
            # 전체 정지 (SPACE / X)
            # ==========================================
            elif key in [' ', 'x', 'X']:
                node.stop_all_motors()

            # Ctrl+C 종료
            elif key == '\x03':
                node.running = False
                break
        else:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = FastMultiMotorKeyboardNode()
    settings = termios.tcgetattr(sys.stdin)

    input_thread = threading.Thread(target=key_loop, args=(node, settings))
    input_thread.daemon = True
    input_thread.start()

    try:
        while node.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except Exception:
        pass
    finally:
        node.target_speed_3 = 0
        node.target_speed_4 = 0
        node.publish_cmd()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n\n테스트를 안전하게 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
