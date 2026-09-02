#!/usr/bin/env python3
import sys
import select
import tty
import termios
import threading
import time
import math
import json
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu, LaserScan
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

class TopicBasedSequentialControlNode(Node):
    # 제어 단계 정의 (State Machine)
    STEP_IDLE = 0
    STEP_1_COMBINED = 1        # 모터 6(위치) + 모터 3(속도) + 구동부 정회전
    STEP_2_ALIGN_OBJECT = 2     # YOLO 기반 구동부 전/후진 지속 정렬
    STEP_3_REVERSE_RESTORE = 3  # 모터 3(속도) + 구동부 역회전 (원상 복구)
    STEP_4_LIDAR_ALIGN = 4      # LiDAR 기반 평행 정렬 및 38cm 도킹
    # 필요 시 STEP_5, STEP_6 추가 정의 가능

    def __init__(self):
        super().__init__('topic_based_sequential_control_node')
        
        self.step_status_pub = self.create_publisher(String, '/step_status', 10)

        # 퍼블리셔 설정
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.motor_vel_pub = self.create_publisher(JointState, 'motor_cmd_vel', 10)
        self.motor_pos_pub = self.create_publisher(JointState, 'motor_cmd_pos', 10)

        # 서브스크라이버 설정
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )
        
        self.joint_sub = self.create_subscription(JointState, 'motor_encoder_positions', self.joint_callback, 10)
        self.yolo_sub = self.create_subscription(String, '/buyr_YOLO', self.yolo_callback, 10)
        self.step_cmd_sub = self.create_subscription(String, '/step_cmd', self.step_cmd_callback, 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            qos_profile_sensor_data
        )

        # 파라미터 설정
        self.MOTOR6_POS_STEP1 = 600
        self.MOTOR6_SPEED = 40
        self.MOTOR3_SPEED = 400.0
        self.TARGET_MOTOR3_TICKS = 23000.0
        
        self.TARGET_ANGULAR_VEL = 0.15
        self.TARGET_ODOM_ANGLE = math.pi / 2.0

        self.ALIGN_MAX_SPEED = 0.04
        self.ALIGN_MIN_SPEED = 0.01
        self.CENTER_MARGIN = 20.0
        self.SLOWDOWN_MARGIN = 100.0

        self.LIDAR_TARGET_DIST = 0.41
        self.LIDAR_SEARCH_ANGLE = math.radians(20)
        self.LIDAR_DIST_MARGIN = 0.01
        self.LIDAR_ANGLE_MARGIN = math.radians(1.0)

        self.LIDAR_MAX_LIN_SPEED = 0.03
        self.LIDAR_MIN_LIN_SPEED = 0.01
        self.LIDAR_MAX_ANG_SPEED = 0.08

        # 상태 및 제어 플래그
        self.current_step = self.STEP_IDLE
        self.current_mode = 'NONE'             # '3STEP' 또는 '6STEP' 구분
        self.auto_sequence = False
        self.cmd_sent = False

        self.current_yaw = 0.0
        self.last_yaw = 0.0
        self.accumulated_yaw = 0.0

        self.current_m3_pos = None
        self.accumulated_m3_ticks = 0.0

        self.motor3_done = False
        self.odom_done = False

        self.latest_yolo_det = None
        self.latest_scan = None

        self.running = True
        self.step_start_time = 0.0
        self.SAFETY_TIMEOUT = 20.0

        self.timer = self.create_timer(0.05, self.control_loop)
        self.print_instructions()

    def print_instructions(self):
        print("\n" + "="*60)
        print("    🚀 [토픽 기반 제어 노드 (3/6단계 통합 지원)]")
        print("="*60)
        print("   수신 제어 토픽: '/step_cmd' (std_msgs/msg/String)")
        print("   - [3단계 모드]: STEP1_3, STEP2_3, STEP3_3")
        print("   - [6단계 모드]: STEP1_6, STEP2_6, STEP3_6, STEP4_6")
        print("   - 공통 명령 : AUTO_3, AUTO_6, STOP")
        print("="*60 + "\n")

    def publish_status(self, status_str):
        msg = String()
        msg.data = status_str
        self.step_status_pub.publish(msg)
        self.get_logger().info(f"📢 [완료 토픽 발행]: {status_str}")

    def step_cmd_callback(self, msg):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"📩 [토픽 명령 수신]: '{cmd}'")

        # --- [3단계 모드 명령어] ---
        if cmd in ['STEP_1', 'STEP1_3']:
            self.current_mode = '3STEP'
            self.start_step_1(auto_next=False)
        elif cmd in ['STEP_2', 'STEP2_3']:
            self.current_mode = '3STEP'
            self.start_step_2(auto_next=False)
        elif cmd in ['STEP_3', 'STEP3_3']:
            self.current_mode = '3STEP'
            self.start_step_3()

        # --- [6단계 모드 명령어] ---
        elif cmd == 'STEP1_6':
            self.current_mode = '6STEP'
            self.start_step_1(auto_next=False)
        elif cmd == 'STEP2_6':
            self.current_mode = '6STEP'
            self.start_step_2(auto_next=False)
        elif cmd == 'STEP3_6':
            self.current_mode = '6STEP'
            self.start_step_3()
        elif cmd == 'STEP4_6':
            self.current_mode = '6STEP'
            self.start_step_4(auto_next=False)

        # --- [자동 시퀀스 및 정지] ---
        elif cmd == 'AUTO_3':
            self.current_mode = '3STEP'
            self.start_auto_sequence()
        elif cmd == 'AUTO_6':
            self.current_mode = '6STEP'
            self.start_auto_sequence()
        elif cmd == 'STOP':
            self.stop_all()

    def scan_callback(self, msg):
        self.latest_scan = msg

    def euler_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def imu_callback(self, msg):
        q = msg.orientation
        self.current_yaw = self.euler_from_quaternion(q)

        if self.current_step in [self.STEP_1_COMBINED, self.STEP_3_REVERSE_RESTORE]:
            diff = self.current_yaw - self.last_yaw
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            self.accumulated_yaw += abs(diff)

        self.last_yaw = self.current_yaw

    def joint_callback(self, msg):
        if 'motor_3' in msg.name:
            idx = msg.name.index('motor_3')
            new_m3_pos = msg.position[idx]

            if self.current_m3_pos is None:
                self.current_m3_pos = new_m3_pos

            if self.current_step in [self.STEP_1_COMBINED, self.STEP_3_REVERSE_RESTORE]:
                diff = new_m3_pos - self.current_m3_pos
                if diff > 32767:
                    diff -= 65536
                elif diff < -32767:
                    diff += 65536
                self.accumulated_m3_ticks += abs(diff)

            self.current_m3_pos = new_m3_pos

    def yolo_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.latest_yolo_det = data[0] if data else None
        except Exception as e:
            self.get_logger().error(f"YOLO 파싱 오류: {e}")

    def get_table_distance(self):
        if self.latest_scan is None:
            return None, None

        scan = self.latest_scan
        points = []

        for i, angle in enumerate(scan.ranges):
            curr_angle = scan.angle_min + i * scan.angle_increment
            if abs(curr_angle) <= self.LIDAR_SEARCH_ANGLE:
                r = scan.ranges[i]
                if scan.range_min < r < scan.range_max and not math.isinf(r) and not math.isnan(r):
                    x = r * math.cos(curr_angle)
                    y = r * math.sin(curr_angle)
                    points.append((x, y))

        if len(points) < 10:
            return None, None

        avg_x = sum(p[0] for p in points) / len(points)
        valid_points = [p for p in points if abs(p[0] - avg_x) < 0.15]

        if len(valid_points) < 8:
            return None, None

        n = len(valid_points)
        sum_y = sum(p[1] for p in valid_points)
        sum_x = sum(p[0] for p in valid_points)
        sum_yy = sum(p[1]**2 for p in valid_points)
        sum_yx = sum(p[1]*p[0] for p in valid_points)

        denominator = (n * sum_yy - sum_y ** 2)
        if abs(denominator) < 1e-6:
            return None, None

        a = (n * sum_yx - sum_y * sum_x) / denominator
        c = (sum_x - a * sum_y) / n

        angle = math.atan(a)
        return c, angle

    def reset_step_variables(self):
        self.last_yaw = self.current_yaw
        self.accumulated_yaw = 0.0
        self.accumulated_m3_ticks = 0.0
        self.motor3_done = False
        self.odom_done = False
        self.cmd_sent = False
        self.step_start_time = time.time()

    def start_auto_sequence(self):
        if self.current_step != self.STEP_IDLE:
            self.get_logger().warn("⚠️ 이미 다른 동작이 진행 중입니다.")
            return

        self.get_logger().info(f"▶️ [{self.current_mode} 자동 시퀀스 시작]")
        self.start_step_1(auto_next=True)

    def start_step_1(self, auto_next=False):
        if self.current_m3_pos is None:
            self.get_logger().error("❌ 모터3 엔코더 피드백이 준비되지 않았습니다!")
            return

        self.auto_sequence = auto_next
        self.reset_step_variables()
        self.current_step = self.STEP_1_COMBINED
        self.get_logger().info("▶️ [Step 1 시작] 정회전 동작")

    def start_step_2(self, auto_next=False):
        self.auto_sequence = auto_next
        self.step_start_time = time.time()
        self.current_step = self.STEP_2_ALIGN_OBJECT
        self.get_logger().info("▶️ [Step 2 시작] YOLO 미세 정렬 진행 중...")

    def start_step_3(self):
        if self.current_m3_pos is None:
            self.get_logger().error("❌ 모터3 엔코더 피드백이 준비되지 않았습니다!")
            return

        self.reset_step_variables()
        self.current_step = self.STEP_3_REVERSE_RESTORE
        self.get_logger().info("▶️ [Step 3 시작] 역회전으로 포즈 원상 복구 중...")

    def start_step_4(self, auto_next=False):
        self.auto_sequence = auto_next
        self.step_start_time = time.time()
        self.current_step = self.STEP_4_LIDAR_ALIGN
        self.get_logger().info("▶️ [Step 4 시작] LiDAR 정렬")

    def stop_all(self):
        self.current_step = self.STEP_IDLE
        self.auto_sequence = False
        self.cmd_sent = False
        self.publish_motor_vel({'motor_3': 0.0})
        self.publish_cmd_vel(0.0, 0.0)
        self.get_logger().info("🛑 전체 제어 정지 완료")

    def control_loop(self):
        if self.current_step == self.STEP_IDLE:
            return

        elapsed_time = time.time() - self.step_start_time

        # --- [Step 1] 정회전 동작 ---
        if self.current_step == self.STEP_1_COMBINED:
            if not self.motor3_done and self.accumulated_m3_ticks >= self.TARGET_MOTOR3_TICKS:
                self.motor3_done = True
            if not self.odom_done and self.accumulated_yaw >= self.TARGET_ODOM_ANGLE:
                self.odom_done = True

            if not self.cmd_sent:
                self.publish_motor_pos({'motor_6': self.MOTOR6_POS_STEP1}, velocity=self.MOTOR6_SPEED)
                self.cmd_sent = True

            active_m3 = -self.MOTOR3_SPEED if not self.motor3_done else 0.0
            self.publish_motor_vel({'motor_3': active_m3})

            active_ang = self.TARGET_ANGULAR_VEL if not self.odom_done else 0.0
            self.publish_cmd_vel(0.0, active_ang)

            if (self.motor3_done and self.odom_done) or (elapsed_time >= self.SAFETY_TIMEOUT):
                self.publish_motor_vel({'motor_3': 0.0})
                self.publish_cmd_vel(0.0, 0.0)

                # Step 1 완료 토픽 발행
                self.publish_status('step1_complete')

                if self.auto_sequence:
                    self.start_step_2(auto_next=True)
                else:
                    self.stop_all()

        # --- [Step 2] YOLO 미세 정렬 ---
        elif self.current_step == self.STEP_2_ALIGN_OBJECT:
            if self.latest_yolo_det is None:
                self.publish_cmd_vel(0.0, 0.0)
                return

            cx = self.latest_yolo_det['cx']
            img_w = self.latest_yolo_det['img_w']
            diff_x = cx - (img_w / 2.0)
            abs_diff = abs(diff_x)

            if abs_diff <= self.CENTER_MARGIN:
                self.publish_cmd_vel(0.0, 0.0)
                
                # Step 2 완료 토픽 발행
                self.publish_status('step2_complete')

                if self.auto_sequence:
                    if self.current_mode == '6STEP':
                        self.start_step_3()
                    else:
                        self.start_step_4(auto_next=True)
                else:
                    self.stop_all()
            else:
                if abs_diff >= self.SLOWDOWN_MARGIN:
                    target_speed = self.ALIGN_MAX_SPEED
                else:
                    ratio = (abs_diff - self.CENTER_MARGIN) / (self.SLOWDOWN_MARGIN - self.CENTER_MARGIN)
                    target_speed = self.ALIGN_MIN_SPEED + (self.ALIGN_MAX_SPEED - self.ALIGN_MIN_SPEED) * ratio

                cmd_linear_x = target_speed if diff_x > 0 else -target_speed
                self.publish_cmd_vel(cmd_linear_x, 0.0)

        # --- [Step 3] 역회전 (원상 복구) ---
        elif self.current_step == self.STEP_3_REVERSE_RESTORE:
            if not self.motor3_done and self.accumulated_m3_ticks >= self.TARGET_MOTOR3_TICKS:
                self.motor3_done = True
            if not self.odom_done and self.accumulated_yaw >= self.TARGET_ODOM_ANGLE:
                self.odom_done = True

            active_m3 = self.MOTOR3_SPEED if not self.motor3_done else 0.0
            self.publish_motor_vel({'motor_3': active_m3})

            active_ang = -self.TARGET_ANGULAR_VEL if not self.odom_done else 0.0
            self.publish_cmd_vel(0.0, active_ang)

            if (self.motor3_done and self.odom_done) or (elapsed_time >= self.SAFETY_TIMEOUT):
                self.publish_motor_vel({'motor_3': 0.0})
                self.publish_cmd_vel(0.0, 0.0)

                # Step 3 완료 토픽 발행
                self.publish_status('step3_complete')

                if self.auto_sequence and self.current_mode == '6STEP':
                    self.start_step_4(auto_next=True)
                else:
                    self.stop_all()

        # --- [Step 4] LiDAR 평행 정렬 ---
        elif self.current_step == self.STEP_4_LIDAR_ALIGN:
            dist, angle = self.get_table_distance()

            if dist is None or angle is None:
                self.publish_cmd_vel(0.0, 0.0)
                return

            cmd = Twist()
            dist_err = dist - self.LIDAR_TARGET_DIST

            if abs(angle) > self.LIDAR_ANGLE_MARGIN:
                target_ang = -angle * 0.4
                cmd.angular.z = math.copysign(
                    min(abs(target_ang), self.LIDAR_MAX_ANG_SPEED), 
                    target_ang
                )
                cmd.linear.x = 0.0
            else:
                cmd.angular.z = 0.0
                if abs(dist_err) > self.LIDAR_DIST_MARGIN:
                    calc_speed = dist_err * 0.3
                    abs_speed = max(min(abs(calc_speed), self.LIDAR_MAX_LIN_SPEED), self.LIDAR_MIN_LIN_SPEED)
                    cmd.linear.x = math.copysign(abs_speed, dist_err)
                else:
                    cmd.linear.x = 0.0

            if abs(dist_err) <= self.LIDAR_DIST_MARGIN and abs(angle) <= self.LIDAR_ANGLE_MARGIN:
                self.publish_cmd_vel(0.0, 0.0)
                
                # Step 4 완료 토픽 발행
                self.publish_status('step4_complete')
                self.stop_all()
            else:
                self.publish_cmd_vel(cmd.linear.x, cmd.angular.z)

    def publish_motor_vel(self, motor_speeds_dict):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(motor_speeds_dict.keys())
        msg.velocity = [float(v) for v in motor_speeds_dict.values()]
        self.motor_vel_pub.publish(msg)

    def publish_motor_pos(self, motor_pos_dict, velocity=0):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(motor_pos_dict.keys())
        msg.position = [float(p) for p in motor_pos_dict.values()]
        if velocity > 0:
            msg.velocity = [float(velocity)]
        self.motor_pos_pub.publish(msg)

    def publish_cmd_vel(self, linear_x, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist_msg)


def key_loop(node, settings):
    while node.running and rclpy.ok():
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

            if key == '1':
                node.step_cmd_callback(String(data='STEP1_3'))
            elif key == '2':
                node.step_cmd_callback(String(data='STEP2_3'))
            elif key == '3':
                node.step_cmd_callback(String(data='STEP3_3'))
            elif key == '4':
                node.step_cmd_callback(String(data='STEP4_6'))
            elif key in ['s', 'S']:
                node.stop_all()
            elif key == '\x03':
                node.running = False
                break
        else:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = TopicBasedSequentialControlNode()
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
