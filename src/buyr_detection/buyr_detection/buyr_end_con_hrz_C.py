#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String


class Step5EncoderControlNode(Node):

    def __init__(self):
        super().__init__('step5_encoder_control_node')

        # 1. Publisher 설정
        self.arduino_pwm_pub_ = self.create_publisher(Int32, '/arduino_pwm', 10)
        self.vel_pub_ = self.create_publisher(JointState, 'motor_cmd_vel', 10)
        self.pos_pub_ = self.create_publisher(JointState, 'motor_cmd_pos', 10)
        self.step_status_pub = self.create_publisher(String, '/step_status', 10)

        # 2. Subscriber 설정
        self.step_sub_ = self.create_subscription(
            String, '/step_cmd', self.step_cmd_callback, 10
        )

        self.pwm_sub_ = self.create_subscription(
            Int32, '/arduino_pwm_cmd', self.pwm_cmd_callback, 10
        )

        self.encoder_topic_name = 'motor_encoder_positions'
        self.joint_sub_ = self.create_subscription(
            JointState, self.encoder_topic_name, self.joint_state_callback, 10
        )

        # 3. 제어 상태 변수
        self.current_m4_pos = None
        self.m4_start_pos = None  # STEP_5 시작 시 원점
        self.m4_target_pos = None
        self.m4_command_speed = 0.0
        self.M4_MAX_SPEED = 400.0
        self.M4_MIN_SPEED = 200.0
        self.M4_TOLERANCE = 15.0

        # 16비트 엔코더 범주 상수
        self.ENC_MAX = 65536
        self.ENC_HALF = 32768

        # 모터 위치 제어 변수 (3, 4, 5, 6, 7번 모터)
        self.current_positions = {3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0, 7: 0.0}

        # 5번 Lift 모터 원점 동적 관리 변수 및 오프셋 설정
        self.m5_base_pos = None
        self.m5_initialized = False
        self.LIFT_DISCHARGE_OFFSET = 750.0  # 상대 이동 오프셋
        self.LIFT_DISCHARGE_SPEED = 40.0

        self.TILT_VISION_POS = 1700.0
        self.TILT_SPEED = 120.0

        self.YAW_DISCHARGE_DELTA = -44500.0
        self.target_yaw_pos = 0.0
        self.yaw_return_pos = 0.0

        # 시퀀스 상태 변수 ('IDLE', 'MOVING_FORWARD', 'MOVING_BACK', 'STEP6_...')
        self.sequence_state = 'IDLE'
        self.state_start_time = time.time()
        self.cmd_sent = False

        # 20Hz (0.05s) 제어 루프
        self.timer = self.create_timer(0.05, self.control_loop)
        self.print_instructions()

    @property
    def lift_top_pos(self):
        base = self.m5_base_pos if self.m5_base_pos is not None else 0.0
        return base

    @property
    def lift_discharge_pos(self):
        base = self.m5_base_pos if self.m5_base_pos is not None else 0.0
        return base + self.LIFT_DISCHARGE_OFFSET

    def print_instructions(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 STEP 5 / STEP 6 전용 제어 노드 (6단계 모드 호환)')
        self.get_logger().info('수신 명령어: STEP5_6, STEP6_6, STOP')
        self.get_logger().info('=' * 60)

    def publish_status(self, status_str):
        """마스터 노드로 동작 완료 상태 토픽 전달"""
        msg = String()
        msg.data = status_str
        self.step_status_pub.publish(msg)
        self.get_logger().info(f'📢 [완료 토픽 발행]: {status_str}')

    def send_arduino_pwm(self, pwm_val):
        """아두이노 브릿지 노드(/arduino_pwm)로 PWM 값을 퍼블리시"""
        msg = Int32()
        msg.data = int(pwm_val)
        self.arduino_pwm_pub_.publish(msg)
        self.get_logger().info(f'📡 [/arduino_pwm] 토픽 발행: {pwm_val}')

    def pwm_cmd_callback(self, msg):
        pwm_val = msg.data
        self.get_logger().warn(
            f'🛡️ [안전장치 수신] 외부 토픽으로 PWM 제어 요청: {pwm_val}'
        )
        self.send_arduino_pwm(pwm_val)

    def get_shortest_encoder_error(self, target, current):
        """16비트 엔코더 최단 경로 오차 보정 (-32768 ~ +32767)"""
        target = target % self.ENC_MAX
        current = current % self.ENC_MAX
        diff = target - current

        if diff > self.ENC_HALF:
            diff -= self.ENC_MAX
        elif diff < -self.ENC_HALF:
            diff += self.ENC_MAX

        return diff

    def set_state(self, new_state):
        self.sequence_state = new_state
        self.state_start_time = time.time()
        self.cmd_sent = False

    def joint_state_callback(self, msg):
        # 1. 이름 기반 데이터 파싱
        if msg.name:
            for name, pos in zip(msg.name, msg.position):
                try:
                    m_id = int(name.split('_')[-1])
                    if m_id in self.current_positions:
                        self.current_positions[m_id] = float(pos % self.ENC_MAX)

                        if m_id == 4:
                            self.current_m4_pos = self.current_positions[4]

                        if m_id == 5 and not self.m5_initialized:
                            self.m5_base_pos = float(pos)
                            self.m5_initialized = True
                            self.get_logger().info(
                                f'🎯 [5번 Lift 모터] 원점 등록 완료 ({self.m5_base_pos})'
                            )
                except (ValueError, IndexError):
                    pass

        # 2. 이름 배열이 없는 예외적 JointState 매핑 지원
        elif len(msg.position) >= 4:
            self.current_m4_pos = float(msg.position[3] % self.ENC_MAX)
            self.current_positions[4] = self.current_m4_pos

    def step_cmd_callback(self, msg):
        cmd = msg.data.strip().upper()

        # Step 5 시작 명령 수신 (STEP5_6, STEP_5 등)
        if cmd in ['STEP5_6', 'STEP_5', 'STEP5']:
            if self.sequence_state != 'IDLE':
                self.get_logger().warn('⚠️ 이미 다른 시퀀스가 실행 중입니다.')
                return

            if self.current_m4_pos is None:
                self.get_logger().error('❌ 모터4 엔코더 피드백 수신 대기 중...')
                return

            self.get_logger().info('🚀 [STEP 5 시작] PWM 128 ON 및 모터4 전진 시작')
            self.send_arduino_pwm(128)

            self.m4_start_pos = self.current_m4_pos
            self.m4_target_pos = (self.m4_start_pos + 6000) % self.ENC_MAX
            self.set_state('MOVING_FORWARD')

        # Step 6 시작 명령 수신 (STEP6_6, STEP_6 등)
        elif cmd in ['STEP6_6', 'STEP_6', 'STEP6']:
            if self.sequence_state != 'IDLE':
                self.get_logger().warn('⚠️ 이미 다른 시퀀스가 실행 중입니다.')
                return

            self.yaw_return_pos = self.current_positions[3]
            self.target_yaw_pos = (
                self.yaw_return_pos + self.YAW_DISCHARGE_DELTA
            ) % self.ENC_MAX

            self.get_logger().info('🚀 [STEP 6 시작] 배출 시퀀스 시작')
            self.set_state('STEP6_DISCHARGE_ROTATE')

        elif cmd in ['PWM_OFF', 'PWM0']:
            self.send_arduino_pwm(0)

        elif cmd in ['STOP', 'HALT']:
            self.stop_all()

    def control_loop(self):
        if self.sequence_state == 'IDLE':
            return

        now = time.time()
        elapsed_time = now - self.state_start_time

        cmd_vel_3 = 0.0
        cmd_vel_4 = 0.0
        cmd_pos_5 = None
        cmd_vel_5 = 0.0
        cmd_pos_6 = None
        cmd_vel_6 = 0.0

        # --------------------------------------------------
        # STEP 5: 4번 모터 자동 왕복 제어
        # --------------------------------------------------
        if self.sequence_state in ['MOVING_FORWARD', 'MOVING_BACK']:
            if self.m4_target_pos is not None and self.current_m4_pos is not None:
                pos_error = self.get_shortest_encoder_error(
                    self.m4_target_pos, self.current_m4_pos
                )

                if abs(pos_error) <= self.M4_TOLERANCE or elapsed_time > 10.0:
                    if self.sequence_state == 'MOVING_FORWARD':
                        self.get_logger().info(
                            '📍 [STEP 5] 전진 완료 -> 복귀 동작 시작'
                        )
                        self.m4_target_pos = self.m4_start_pos
                        self.set_state('MOVING_BACK')

                    elif self.sequence_state == 'MOVING_BACK':
                        self.get_logger().info('🏁 [STEP 5 완료] 원점 복귀 완료')
                        self.m4_command_speed = 0.0
                        self.m4_target_pos = None
                        cmd_vel_4 = 0.0
                        self.set_state('IDLE')

                        # 마스터 노드로 Step 5 완료 토픽 발행
                        self.publish_status('step5_complete')
                else:
                    kp = 0.6
                    calc_speed = pos_error * kp

                    if calc_speed > 0:
                        speed = min(
                            self.M4_MAX_SPEED, max(self.M4_MIN_SPEED, float(calc_speed))
                        )
                    else:
                        speed = max(
                            -self.M4_MAX_SPEED, min(-self.M4_MIN_SPEED, float(calc_speed))
                        )

                    self.m4_command_speed = speed
                    cmd_vel_4 = float(self.m4_command_speed)

        # --------------------------------------------------
        # STEP 6: 배출 동작 시퀀스
        # --------------------------------------------------
        elif self.sequence_state == 'STEP6_DISCHARGE_ROTATE':
            e3 = self.get_shortest_encoder_error(
                self.target_yaw_pos, self.current_positions[3]
            )
            if abs(e3) < 400 or elapsed_time > 12.0:
                cmd_vel_3 = 0.0
                self.get_logger().info(
                    f'✅ [STEP 6] 배출구 회전 완료 -> Tilt 위치({self.TILT_VISION_POS}) 복귀'
                )
                self.set_state('STEP6_RESET_TILT')
            else:
                cmd_vel_3 = -800.0

        elif self.sequence_state == 'STEP6_RESET_TILT':
            cmd_pos_6 = self.TILT_VISION_POS
            cmd_vel_6 = self.TILT_SPEED
            if elapsed_time >= 1.0:
                self.set_state('STEP6_LIFT_DOWN')

        elif self.sequence_state == 'STEP6_LIFT_DOWN':
            cmd_pos_5 = self.lift_discharge_pos
            cmd_vel_5 = self.LIFT_DISCHARGE_SPEED
            cmd_pos_6 = self.TILT_VISION_POS
            cmd_vel_6 = self.TILT_SPEED

            if (
                abs(self.current_positions[5] - self.lift_discharge_pos) < 100
                or elapsed_time > 2.5
            ):
                self.send_arduino_pwm(0)
                self.set_state('STEP6_LIFT_UP')

        elif self.sequence_state == 'STEP6_LIFT_UP':
            cmd_pos_5 = self.lift_top_pos
            cmd_vel_5 = self.LIFT_DISCHARGE_SPEED
            cmd_pos_6 = self.TILT_VISION_POS
            cmd_vel_6 = self.TILT_SPEED

            if (
                abs(self.current_positions[5] - self.lift_top_pos) < 100
                or elapsed_time > 2.5
            ):
                self.set_state('STEP6_RESET_YAW')

        elif self.sequence_state == 'STEP6_RESET_YAW':
            cmd_pos_5 = self.lift_top_pos
            cmd_vel_5 = self.LIFT_DISCHARGE_SPEED
            cmd_pos_6 = self.TILT_VISION_POS
            cmd_vel_6 = self.TILT_SPEED

            e3 = self.get_shortest_encoder_error(
                self.yaw_return_pos, self.current_positions[3]
            )

            if abs(e3) < 400 or elapsed_time > 12.0:
                cmd_vel_3 = 0.0
                self.set_state('IDLE')

                # 마스터 노드로 Step 6 완료 토픽 발행
                self.publish_status('step6_complete')
            else:
                cmd_vel_3 = 800.0

        else:
            self.m4_command_speed = 0.0
            cmd_vel_4 = 0.0

        # --------------------------------------------------
        # ROS 2 토픽 퍼블리시
        # --------------------------------------------------
        # 1. 속도 제어 모터 (3, 4번)
        vel_msg = JointState()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.name = ['motor_3', 'motor_4']
        vel_msg.velocity = [float(cmd_vel_3), float(cmd_vel_4)]
        self.vel_pub_.publish(vel_msg)

        # 2. 위치 제어 모터 (5, 6번)
        if cmd_pos_5 is not None or cmd_pos_6 is not None:
            pos_msg = JointState()
            pos_msg.header.stamp = self.get_clock().now().to_msg()

            names = []
            positions = []
            velocities = []

            if cmd_pos_5 is not None:
                names.append('motor_5')
                positions.append(float(cmd_pos_5))
                velocities.append(float(cmd_vel_5))

            if cmd_pos_6 is not None:
                names.append('motor_6')
                positions.append(float(cmd_pos_6))
                velocities.append(float(cmd_vel_6))

            pos_msg.name = names
            pos_msg.position = positions
            pos_msg.velocity = velocities
            self.pos_pub_.publish(pos_msg)

    def stop_all(self):
        self.m4_target_pos = None
        self.m4_command_speed = 0.0
        self.sequence_state = 'IDLE'

        # 모든 모터 정지 명령 전달
        vel_msg = JointState()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.name = ['motor_3', 'motor_4']
        vel_msg.velocity = [0.0, 0.0]
        self.vel_pub_.publish(vel_msg)

        self.send_arduino_pwm(0)
        self.get_logger().warn('🛑 모든 동작 정지 및 PWM OFF 요청 완료')

    def destroy_node(self):
        self.stop_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Step5EncoderControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
