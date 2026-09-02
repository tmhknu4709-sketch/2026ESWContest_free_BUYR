#!/usr/bin/env python3
import sys
import select
import tty
import termios
import threading
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class CombinedTurnTeleopNode(Node):
    def __init__(self):
        super().__init__('combined_turn_teleop_node')

        # 1. 퍼블리셔 & 서브스크라이버 설정
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.motor_vel_pub = self.create_publisher(JointState, 'motor_cmd_vel', 10)

        # EKF 필터링된 오도메트리 구독
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.joint_sub = self.create_subscription(JointState, 'motor_encoder_positions', self.joint_callback, 10)

        # 2. 파라미터 설정
        self.YAW_MAX_SPEED = 400.0          # 3번 모터 속도
        self.TARGET_ANGULAR_VEL = 0.15       # rad/s (구동부 속도)
        self.TARGET_ODOM_ANGLE = math.pi / 2.0  # 구동부 90도 (rad)
        self.TARGET_MOTOR3_TICKS = 21300.0   # 3번 모터 목표 틱 수

        # 3. 상태 및 측정 변수 초기화 (에러 방지)
        self.current_yaw = 0.0
        self.last_yaw = 0.0
        self.accumulated_yaw = 0.0          # 라디안 단위 누적 회전량

        self.current_m3_pos = None
        self.accumulated_m3_ticks = 0.0     # 모터3 누적 틱 수

        self.target_motor_speed = 0.0
        self.target_angular_vel = 0.0
        
        self.is_turning = False
        self.running = True
        self.turn_start_time = 0.0
        self.SAFETY_TIMEOUT = 15.0          # 안전 타임아웃 (15초)

        self.odom_done = False
        self.motor3_done = False

        # 4. 20Hz 제어 루프 타이머 (0.05초 간격)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.print_instructions()

    def print_instructions(self):
        print("\n" + "="*60)
        print("    🚀 [통합 회전 원격 제어 노드 (EKF Odom + 모터3 피드백)]")
        print("="*60)
        print(f"   - 3번 모터 목표 이동량: {int(self.TARGET_MOTOR3_TICKS)} Ticks")
        print(f"   - 구동부 목표 회전량: 90.0° ({self.TARGET_ANGULAR_VEL} rad/s)")
        print("   - [Q] : 3번 모터 CW (-500) + 구동부 CCW (+0.15 rad/s, 90°)")
        print("   - [W] : 3번 모터 CCW (+500) + 구동부 CW (-0.15 rad/s, 90°)")
        print("   - [S] : 즉시 정지")
        print("   - [Ctrl+C] : 종료")
        print("="*60 + "\n")

    def euler_from_quaternion(self, q):
        """Quaternion -> Euler Yaw(rad) 변환"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """구동부 Body 회전각 계산 (시작 각도 대비 차이 누적)"""
        q = msg.pose.pose.orientation
        self.current_yaw = self.euler_from_quaternion(q)

        if self.is_turning:
            diff = self.current_yaw - self.last_yaw
            # -pi ~ pi 변환 경계(Rollover) 처리
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            self.accumulated_yaw += abs(diff)

        self.last_yaw = self.current_yaw

    def joint_callback(self, msg):
        """3번 모터 엔코더 피드백 수신 (0~65535 Rollover 고려)"""
        if 'motor_3' in msg.name:
            idx = msg.name.index('motor_3')
            new_pos = msg.position[idx]

            # 최초 수신 시 초기화
            if self.current_m3_pos is None:
                self.current_m3_pos = new_pos
                return

            if self.is_turning:
                diff = new_pos - self.current_m3_pos
                # 0 ~ 65535 Rollover 처리
                if diff > 32767:
                    diff -= 65536
                elif diff < -32767:
                    diff += 65536
                
                self.accumulated_m3_ticks += abs(diff)

            self.current_m3_pos = new_pos

    def start_turn(self, q_pressed=True):
        if self.is_turning:
            self.get_logger().warn("⚠️ 이미 회전 동작 수행 중입니다.")
            return

        if self.current_m3_pos is None:
            self.get_logger().error("❌ 모터3 엔코더 피드백이 수신되지 않아 동작을 시작할 수 없습니다!")
            return

        # 기준점 및 누적치 초기화
        self.last_yaw = self.current_yaw
        self.accumulated_yaw = 0.0
        self.accumulated_m3_ticks = 0.0

        self.turn_start_time = time.time()

        if q_pressed:
            self.target_motor_speed = -self.YAW_MAX_SPEED    # CW (-)
            self.target_angular_vel = self.TARGET_ANGULAR_VEL # 구동부 CCW (+)
            self.get_logger().info("➡️ [Q] 실행 시작")
        else:
            self.target_motor_speed = self.YAW_MAX_SPEED     # CCW (+)
            self.target_angular_vel = -self.TARGET_ANGULAR_VEL # 구동부 CW (-)
            self.get_logger().info("➡️ [W] 실행 시작")

        self.is_turning = True
        self.odom_done = False
        self.motor3_done = False

    def stop_all(self):
        self.target_motor_speed = 0.0
        self.target_angular_vel = 0.0
        self.is_turning = False
        self.publish_cmd_custom(0.0, 0.0)
        self.get_logger().info("🛑 정지 명령 실행")

    def control_loop(self):
        if not self.is_turning:
            return

        # 1. Odom 90도 달성 체크
        if not self.odom_done:
            if self.accumulated_yaw >= self.TARGET_ODOM_ANGLE:
                self.odom_done = True
                self.get_logger().info("✅ 구동부(Odom) 90° 목표 도달 완료")

        # 2. 모터3 19500 Ticks 달성 체크
        if not self.motor3_done:
            if self.accumulated_m3_ticks >= self.TARGET_MOTOR3_TICKS:
                self.motor3_done = True
                self.get_logger().info("✅ 모터3 19,500 Ticks 목표 도달 완료")

        # 3. 실시간 진행 상황 로깅
        current_deg = math.degrees(self.accumulated_yaw)
        self.get_logger().info(
            f"[회전 중] Odom: {current_deg:.1f}° / 90.0° | 모터3: {int(self.accumulated_m3_ticks)} / {int(self.TARGET_MOTOR3_TICKS)} Ticks"
        )

        # 4. 미달성 파트만 동작 유지, 달성된 파트는 0.0 출력
        active_angular_vel = self.target_angular_vel if not self.odom_done else 0.0
        active_motor_speed = self.target_motor_speed if not self.motor3_done else 0.0

        self.publish_cmd_custom(active_motor_speed, active_angular_vel)

        # 5. 둘 다 완료되었거나 타임아웃 시 완전 정지
        elapsed_time = time.time() - self.turn_start_time
        if (self.odom_done and self.motor3_done) or (elapsed_time >= self.SAFETY_TIMEOUT):
            if elapsed_time >= self.SAFETY_TIMEOUT:
                self.get_logger().warn("⚠️ 안전 타임아웃(15초)으로 인해 강제 정지합니다.")
            
            self.stop_all()
            self.get_logger().info(
                f"✅ [완료] Odom: {current_deg:.1f}°, 모터3: {int(self.accumulated_m3_ticks)} Ticks"
            )

    def publish_cmd_custom(self, motor_speed, angular_vel):
        """모터3 및 구동부 명령 동시 발행"""
        motor_msg = JointState()
        motor_msg.header.stamp = self.get_clock().now().to_msg()
        motor_msg.name = ['motor_3']
        motor_msg.velocity = [float(motor_speed)]
        self.motor_vel_pub.publish(motor_msg)

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = float(angular_vel)
        self.cmd_vel_pub.publish(twist_msg)

    def publish_cmd(self):
        self.publish_cmd_custom(self.target_motor_speed, self.target_angular_vel)


def key_loop(node, settings):
    while node.running and rclpy.ok():
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

            if key in ['q', 'Q']:
                node.start_turn(q_pressed=True)
            elif key in ['w', 'W']:
                node.start_turn(q_pressed=False)
            elif key in ['s', 'S']:
                node.stop_all()
            elif key == '\x03':  # Ctrl+C
                node.running = False
                break
        else:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = CombinedTurnTeleopNode()
    settings = termios.tcgetattr(sys.stdin)

    input_thread = threading.Thread(target=key_loop, args=(node, settings))
    input_thread.daemon = True
    input_thread.start()

    try:
        while node.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except Exception as e:
        node.get_logger().error(f"오류 발생: {e}")
    finally:
        node.stop_all()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n프로그램을 종료합니다.")
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
