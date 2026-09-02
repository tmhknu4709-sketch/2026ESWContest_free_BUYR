#!/usr/bin/env python3
import json
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String

# =========================================================
# [클래스 관리 설정]
# EGG 외의 신규 대상 클래스명을 변경할 경우 아래 변수 1개만 수정하세요.
# (예: 'heim' -> 'bread', 'apple' 등)
# =========================================================
TARGET_CLASS_NAME = 'heim'


class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')

        # ==========================================
        # 1. 파라미터 및 오프셋 설정
        # ==========================================
        # 물품 종류 상태 추적 (기본값: egg)
        self.current_target_class = 'egg'

        self.GRIPPER_OFFSET_Y = 125.0
        
        self.MAX_MOTOR4_POS = 12000.0 #4번 모터 안전거리 제한

        # 5번 Lift 모터 원점 및 relative offset 계산
        self.m5_base_pos = None  # 노드 시작 시 5번 모터의 실제 위치(원점)
        self.m5_initialized = False

        # Class별 Step 2 하강 오프셋 (Egg / Target Class 분리)
        self.LIFT_DOWN_OFFSET_EGG = 3750.0
        self.LIFT_DOWN_OFFSET_CUSTOM = 3600.0  # Custom Target 클래스용 하강 높이

        # Class별 동적 배출 오프셋 기본 설정값
        self.LIFT_DISCHARGE_OFFSET_EGG = 1450.0
        self.LIFT_DISCHARGE_OFFSET_CUSTOM = 750.0

        self.LIFT_SPEED = 100.0  # Lift 속도 제한
        self.LIFT_DISCHARGE_SPEED = 40.0

        # 3번 Yaw 모터 배출 회전량 (Class별)
        self.YAW_DISCHARGE_DELTA_EGG = -42000
        self.YAW_DISCHARGE_DELTA_CUSTOM = -44500

        self.target_yaw_pos = 0.0
        self.yaw_return_pos = 0.0  # 복귀용 위치 저장 변수

        self.TILT_SPEED = 40.0
        self.TILT_VISION_POS = 2050.0  # 비전 인식 / 초기 위치
        self.TILT_DOWN_POS = 1900.0    # Lift 하강 시 Tilt 각도

        # [수정] 5번 Lift 상승 완료 후 전류 안정화를 위한 대기 시간 (초)
        self.LIFT_POST_WAIT_TIME = 0.8  

        # 제어 및 상태 변수
        self.last_target_data = None
        self.state = 'IDLE'
        self.state_start_time = time.time()
        self.cmd_sent = False

        # 엔코더 위치 저장 (3, 4, 5, 6, 7번)
        self.current_positions = {3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0, 7: 0.0}

        self.start_pos_3 = 0.0
        self.start_pos_4 = 0.0

        # ==========================================
        # 2. ROS 2 통신 설정
        # ==========================================
        self.motor_vel_pub = self.create_publisher(
            JointState, '/motor_cmd_vel', 10
        )
        self.motor_pos_pub = self.create_publisher(
            JointState, '/motor_cmd_pos', 10
        )
        self.step_status_pub = self.create_publisher(
            String, '/step_status', 10
        )

        # Arduino PWM 제어용 Publisher
        self.pwm_pub = self.create_publisher(Int32, '/arduino_pwm', 10)

        self.sub_vision = self.create_subscription(
            String, '/buyr_detection_final', self.final_callback, 10
        )
        self.sub_encoder = self.create_subscription(
            JointState, '/motor_encoder_positions', self.encoder_callback, 10
        )
        
        # 통합 마스터 노드의 단계 제어 토픽 수신
        self.sub_step = self.create_subscription(
            String, '/step_cmd', self.step_cmd_callback, 10
        )

        # 제어 루프 타이머 (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'🚀 [MotorControlNode] 시작됨. (커스텀 타겟: {TARGET_CLASS_NAME}) 5번 모터 원점 감지 대기 중...'
        )

    # ==========================================
    # 5번 Lift 모터 동적 목표 위치 반환 동적 속성 (Properties)
    # ==========================================
    @property
    def lift_top_pos(self):
        """현재 원점 기준 최상단(초기) 위치"""
        base = self.m5_base_pos if self.m5_base_pos is not None else 0.0
        return base

    @property
    def lift_down_pos(self):
        """현재 원점 및 target_class에 따른 Step 2 하강(Grip) 위치"""
        base = self.m5_base_pos if self.m5_base_pos is not None else 0.0
        if self.current_target_class == TARGET_CLASS_NAME:
            offset = self.LIFT_DOWN_OFFSET_CUSTOM
        else:
            offset = self.LIFT_DOWN_OFFSET_EGG
        return base + offset

    @property
    def lift_discharge_pos(self):
        """현재 원점 기준 배출 위치 (target_class에 따라 오프셋 변동)"""
        base = self.m5_base_pos if self.m5_base_pos is not None else 0.0
        if self.current_target_class == TARGET_CLASS_NAME:
            offset = self.LIFT_DISCHARGE_OFFSET_CUSTOM
        else:
            offset = self.LIFT_DISCHARGE_OFFSET_EGG
        return base + offset

    @property
    def yaw_discharge_delta(self):
        """target_class에 따른 3번 Yaw 회전 오프셋"""
        if self.current_target_class == TARGET_CLASS_NAME:
            return self.YAW_DISCHARGE_DELTA_CUSTOM
        return self.YAW_DISCHARGE_DELTA_EGG

    # ==========================================
    # 16비트 최단 거리 보정 함수
    # ==========================================
    def get_shortest_encoder_error(self, target, current):
        """0~65535 (0xFFFF) 범위의 16비트 엔코더 오차 보정 함수."""
        target = target % 65536
        current = current % 65536
        diff = target - current

        if diff > 32768:
            diff -= 65536
        elif diff < -32768:
            diff += 65536

        return diff

    # ==========================================
    # 3. 상태 변경 헬퍼 함수
    # ==========================================
    def set_state(self, new_state):
        """상태 변경 시 시간과 명령 상태 플래그를 자동 초기화"""
        self.state = new_state
        self.state_start_time = time.time()
        self.cmd_sent = False

    # ==========================================
    # 4. 토픽 발행 헬퍼 함수
    # ==========================================
    def send_motor_vel(self, motor_id, velocity, motor_id_2=None, velocity_2=None):
        msg = JointState()
        if motor_id_2 is None:
            msg.name = [f'motor_{motor_id}']
            msg.velocity = [float(velocity)]
        else:
            msg.name = [f'motor_{motor_id}', f'motor_{motor_id_2}']
            msg.velocity = [float(velocity), float(velocity_2)]

        self.motor_vel_pub.publish(msg)

    def send_motor_pos(self, motor_id, position, velocity=0):
        msg = JointState()
        msg.name = [f'motor_{motor_id}']
        msg.position = [float(position)]
        if velocity > 0:
            msg.velocity = [float(velocity)]
        self.motor_pos_pub.publish(msg)

    def send_pwm(self, pwm_val):
        """아두이노 PWM 출력 제어 함수"""
        msg = Int32()
        msg.data = int(pwm_val)
        self.pwm_pub.publish(msg)

    def stop_all_vel_motors(self):
        """속도 제어 모터(3, 4번) 동시 정지"""
        self.send_motor_vel(3, 0, 4, 0)

    # ==========================================
    # 5. 콜백 함수
    # ==========================================
    def encoder_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            try:
                m_id = int(name.split('_')[-1])
                self.current_positions[m_id] = float(pos % 65536)

                # 최초 1회 5번 모터 위치를 원점(m5_base_pos)으로 자동 등록
                if m_id == 5 and not self.m5_initialized:
                    self.m5_base_pos = float(pos)
                    self.m5_initialized = True
                    self.get_logger().info(
                        f'🎯 [5번 Lift 모터] 현재 위치({self.m5_base_pos})가 원점으로 등록되었습니다.'
                    )
                    self.send_motor_pos(5, self.lift_top_pos, velocity=self.LIFT_SPEED)
            except ValueError:
                pass

    def final_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.last_target_data = data

            # 리스트 형태나 단일 객체 대응하여 target class 갱신
            if isinstance(data, list) and len(data) > 0:
                self.current_target_class = data[0].get('class', self.current_target_class)
            elif isinstance(data, dict):
                self.current_target_class = data.get('class', self.current_target_class)
        except Exception:
            pass

    def step_cmd_callback(self, msg):
        cmd = msg.data.lower().strip()
        self.get_logger().info(f'📥 단계 명령 수신: [{cmd}] (대상: {self.current_target_class})')

        # [STEP 1] 비전 정렬 명령 수신 (3단계/6단계 공통)
        if cmd in ['step1', 'step1_align']:
            self.stop_all_vel_motors()

            self.start_pos_3 = self.current_positions[3]
            self.start_pos_4 = self.current_positions[4]
            self.get_logger().info(
                f'📌 시작 위치 저장 완료 - 3번: {self.start_pos_3}, 4번: {self.start_pos_4}'
            )

            self.send_motor_pos(7, 2)
            self.send_motor_pos(5, self.lift_top_pos, velocity=self.LIFT_SPEED)
            self.send_motor_pos(6, self.TILT_VISION_POS, velocity=self.TILT_SPEED)

            self.set_state('STEP1_ALIGN')
            self.get_logger().info('▶️ [STEP 1] 정렬 동작 시작')

        # [STEP 2] 하강 및 파지 명령 수신 (3단계/6단계 공통)
        elif cmd in ['step2', 'step2_grip', 'step3_lift_up'] and self.state in ['STEP1_COMPLETE', 'IDLE']:
            self.set_state('STEP2_LIFT_DOWN')
            self.get_logger().info(
                f'▶️ [STEP 2] 하강 및 그리핑/PWM 동작 시작 (Target: {self.current_target_class}, 목표 하강 위치: {self.lift_down_pos})'
            )

        # [STEP 3 / 6] 배출 동작 명령 수신 (3단계 마스터 및 6단계 마스터 수신 호환)
        elif cmd in ['step3', 'step6', 'step_6', 'step4_discharge', 'step5_discharge', 'step6_reset'] and self.state in ['STEP2_COMPLETE', 'IDLE']:
            self.yaw_return_pos = self.current_positions[3]
            self.target_yaw_pos = (
                self.yaw_return_pos + self.yaw_discharge_delta
            ) % 65536

            self.set_state('STEP3_DISCHARGE_ROTATE')
            self.get_logger().info(
                f'▶️ [STEP 3/배출] 배출 동작 시작 (Target: {self.current_target_class.upper()}, Offset: {self.yaw_discharge_delta}, 현재: {self.yaw_return_pos} -> 목표: {self.target_yaw_pos})'
            )

    # ==========================================
    # 6. 메인 제어 루프
    # ==========================================
    def control_loop(self):
        now = time.time()
        elapsed_time = now - self.state_start_time

        # --------------------------------------------------
        # STEP 1: 정렬 동작
        # --------------------------------------------------
        if self.state == 'STEP1_ALIGN':
            if self.last_target_data is None:
                return

            data = self.last_target_data[0] if isinstance(self.last_target_data, list) and len(self.last_target_data) > 0 else self.last_target_data

            ex = data.get('real_x_mm', 0.0)
            ey = data.get('real_y_mm', 0.0)
            ty = ey + self.GRIPPER_OFFSET_Y
            abs_ex, abs_ty = abs(ex), abs(ty)

            if abs_ex <= 10.0 and abs_ty <= 20.0:
                self.stop_all_vel_motors()
                self.set_state('STEP1_COMPLETE')
                self.get_logger().info('✅ [STEP 1 완료] 목표 정렬 완료. 대기 중...')

                status_msg = String()
                status_msg.data = 'step1_complete'
                self.step_status_pub.publish(status_msg)
                return

            vel3 = 0
            if abs_ex > 10.0:
                sx = int(3.0 * ex)
                abs_sx = max(min(abs(sx), 600), 80)
                vel3 = abs_sx if ex > 0 else -abs_sx

            vel4 = 0
            if abs_ty > 20.0:
                sy = int(3.5 * ty)
                abs_sy = max(min(abs(sy), 700), 200)
                
                # 4번 모터 위치가 13000 이상일 경우 전진(ty > 0) 금지
                if ty > 0:
                    if self.current_positions[4] >= self.MAX_MOTOR4_POS:
                        vel4 = 0
                        self.get_logger().warn(
                            f'⚠️ [LIMIT] 4번 모터가 최대 한계점({self.MAX_MOTOR4_POS})에 도달하여 전진을 제한합니다. (현재: {self.current_positions[4]})',
                            throttle_duration_sec=1.0 # 1초에 한 번만 로그 출력
                        )
                    else:
                        vel4 = abs_sy
                else:
                    vel4 = -abs_sy # 후진은 허용

            self.send_motor_vel(3, vel3, 4, vel4)

        # --------------------------------------------------
        # STEP 2: 클래스별(egg / target_class) 하강 및 그리핑/PWM 처리
        # --------------------------------------------------
        elif self.state == 'STEP2_LIFT_DOWN':
            target_down = self.lift_down_pos
            if not self.cmd_sent:
                self.send_motor_pos(6, self.TILT_DOWN_POS, velocity=self.TILT_SPEED)
                self.send_motor_pos(5, target_down, velocity=self.LIFT_SPEED)
                self.cmd_sent = True

            if (abs(self.current_positions[5] - target_down) < 150 or elapsed_time > 3.5):
                self.get_logger().info(f'✅ [{self.current_target_class.upper()}] 하강 및 Tilt 완료 -> 파지 단계로 이동')
                self.set_state('STEP2_GRIP')

        elif self.state == 'STEP2_GRIP':
            if not self.cmd_sent:
                if self.current_target_class == 'egg':
                    self.send_motor_pos(7, 2800)
                    self.get_logger().info('🔒 [EGG] 7번 그리퍼 닫는 중...')
                elif self.current_target_class == TARGET_CLASS_NAME:
                    self.send_pwm(128)
                    self.get_logger().info(f'⚡ [{TARGET_CLASS_NAME.upper()}] PWM 128 출력 시작...')
                self.cmd_sent = True

            if elapsed_time >= 1.0:
                self.get_logger().info(f'✅ [{self.current_target_class.upper()}] 파지/PWM 인가 완료 -> 5번 상승 시작')
                self.set_state('STEP2_LIFT_UP')

        elif self.state == 'STEP2_LIFT_UP':
            target_top = self.lift_top_pos
            if not self.cmd_sent:
                self.send_motor_pos(5, target_top, velocity=self.LIFT_SPEED)
                self.cmd_sent = True

            lift_done = abs(self.current_positions[5] - target_top) < 150

            if lift_done or elapsed_time > 3.5:
                self.get_logger().info('✅ 5번 Lift 상승 완료 -> 전류 안정화 대기 중...')
                self.set_state('STEP2_LIFT_UP_WAIT')

        # [추가된 상태] 5번 Lift 도달 후 역기전력/peak 전류 감소용 대기 단계
        elif self.state == 'STEP2_LIFT_UP_WAIT':
            if elapsed_time >= self.LIFT_POST_WAIT_TIME:
                self.get_logger().info('✅ 대기 완료 -> 3/4번 원점 복귀 시작')
                self.set_state('STEP2_RETURN_POS')

        elif self.state == 'STEP2_RETURN_POS':
            e3 = self.get_shortest_encoder_error(self.start_pos_3, self.current_positions[3])
            e4 = self.get_shortest_encoder_error(self.start_pos_4, self.current_positions[4])

            yaw_done = abs(e3) < 200
            slider_done = abs(e4) < 200

            if (yaw_done and slider_done) or elapsed_time > 4.0:
                self.stop_all_vel_motors()
                self.send_motor_pos(3, self.current_positions[3])
                self.send_motor_pos(4, self.current_positions[4])

                self.set_state('STEP2_COMPLETE')
                self.get_logger().info('✅ [STEP 2 완료] 5번 Lift 및 3/4번 원점 복귀 완료.')

                status_msg = String()
                status_msg.data = 'step2_complete'
                self.step_status_pub.publish(status_msg)
            else:
                vel3 = (1.0 if e3 > 0 else -1.0) * 500 if not yaw_done else 0
                vel4 = (1.0 if e4 > 0 else -1.0) * 500 if not slider_done else 0
                self.send_motor_vel(3, vel3, 4, vel4)

        # --------------------------------------------------
        # STEP 3: 배출 동작 (TARGET_CLASS / EGG 오프셋 반영)
        # --------------------------------------------------
        elif self.state == 'STEP3_DISCHARGE_ROTATE':
            e3 = self.get_shortest_encoder_error(
                self.target_yaw_pos, self.current_positions[3]
            )

            if abs(e3) < 400 or elapsed_time > 12.0:
                self.send_motor_vel(3, 0)
                self.get_logger().info(f'✅ 배출구 회전 완료 -> 6번 Tilt 초기 위치({self.TILT_VISION_POS}) 복귀 시작')
                self.set_state('STEP3_RESET_TILT')
            else:
                self.send_motor_vel(3, -800)

        elif self.state == 'STEP3_RESET_TILT':
            if not self.cmd_sent:
                self.send_motor_pos(6, self.TILT_VISION_POS, velocity=self.TILT_SPEED)
                self.cmd_sent = True

            if elapsed_time >= 1.0:
                self.get_logger().info(f'✅ 6번 Tilt 복귀 완료 -> 5번 Lift 배출 위치 하강 시작 (목표: {self.lift_discharge_pos})')
                self.set_state('STEP3_LIFT_DOWN')

        elif self.state == 'STEP3_LIFT_DOWN':
            target_discharge = self.lift_discharge_pos
            if not self.cmd_sent:
                self.send_motor_pos(5, target_discharge, velocity=self.LIFT_DISCHARGE_SPEED)
                self.send_motor_pos(6, self.TILT_VISION_POS, velocity=self.TILT_SPEED)
                self.cmd_sent = True

            if abs(self.current_positions[5] - target_discharge) < 100 or elapsed_time > 2.5:
                self.get_logger().info(f'✅ 배출 위치 하강 완료 -> [{self.current_target_class.upper()}] 해제 동작 시작')
                self.set_state('STEP3_OPEN_GRIPPER')

        elif self.state == 'STEP3_OPEN_GRIPPER':
            if not self.cmd_sent:
                if self.current_target_class == 'egg':
                    # EGG: 7번 그리퍼 열기
                    self.send_motor_pos(7, 2)
                    self.get_logger().info('🔓 [EGG] 7번 그리퍼 열기...')
                elif self.current_target_class == TARGET_CLASS_NAME:
                    # Target Class: 아두이노 PWM 0 정지
                    self.send_pwm(0)
                    self.get_logger().info(f'⚡ [{TARGET_CLASS_NAME.upper()}] PWM 0 정지 출력...')
                self.cmd_sent = True

            if elapsed_time >= 1.0:
                self.get_logger().info('✅ 해제 완료 -> 5번 Lift 초기 위치 상승')
                self.set_state('STEP3_LIFT_UP')

        elif self.state == 'STEP3_LIFT_UP':
            target_top = self.lift_top_pos
            if not self.cmd_sent:
                self.send_motor_pos(5, target_top, velocity=self.LIFT_DISCHARGE_SPEED)
                self.send_motor_pos(6, self.TILT_VISION_POS, velocity=self.TILT_SPEED)
                self.cmd_sent = True

            if abs(self.current_positions[5] - target_top) < 100 or elapsed_time > 2.5:
                self.get_logger().info('✅ 원점 상승 완료 -> 3번 Yaw 원위치 복귀')
                self.set_state('STEP3_RESET_YAW')

        elif self.state == 'STEP3_RESET_YAW':
            e3 = self.get_shortest_encoder_error(
                self.yaw_return_pos, self.current_positions[3]
            )

            if abs(e3) < 400 or elapsed_time > 12.0:
                self.send_motor_vel(3, 0)
                self.set_state('IDLE')
                self.get_logger().info('🎉 [STEP 3 완료] 반대 회전 복귀 완료. IDLE 전환')

                # 마스터 노드로 완료 상태 전달 (3단계: step3_complete, 6단계: step6_complete 지원)
                status_msg = String()
                status_msg.data = 'step3_complete'
                self.step_status_pub.publish(status_msg)
                
                # 6단계 파이프라인 대응용 신호 추가 발행
                status_msg_6 = String()
                status_msg_6.data = 'step6_complete'
                self.step_status_pub.publish(status_msg_6)
            else:
                self.send_motor_vel(3, 800)

    def destroy_node(self):
        self.stop_all_vel_motors()
        # 노드 종료 시 안전을 위해 PWM 0 인가
        self.send_pwm(0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
