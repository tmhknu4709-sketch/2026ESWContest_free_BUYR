#!/usr/bin/env python3
import math
import time
import signal
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from dynamixel_sdk import *


class MixedMotorControlNode(Node):
    def __init__(self):
        super().__init__('mixed_motor_control_node')

        # 동시 실행을 위한 Callback Group 및 Lock 설정
        self.cb_group = ReentrantCallbackGroup()
        self.dxl_lock = threading.Lock()

        # ==========================================
        # 1. 통신 및 모터 그룹 설정
        # ==========================================
        self.BAUDRATE = 1000000
        self.DEVICENAME = '/dev/ttyUSB_DXL'

        # 모터 분류
        self.MX_VEL_IDS = [1, 2]         # MX-106 (Protocol 2.0) - 구동륜 속도제어
        self.MX_EXT_POS_IDS = [5]        # MX-106 (Protocol 2.0) - Extended Position Control (Multi-turn)
        self.EX_VEL_IDS = [3, 4]         # EX-106 (Protocol 1.0) - 속도제어
        self.EX_POS_IDS = [6, 7]         # EX-106 (Protocol 1.0) - 위치제어 (0~4095)
        self.ALL_IDS = [1, 2, 3, 4, 5, 6, 7]

        # 로봇 물리 파라미터 (1, 2번 구동륜 오도메트리용)
        self.wheel_radius = 0.075
        self.wheel_separation = 0.5
        self.TICKS_PER_REV = 4096.0
        self.VELOCITY_CONSTANT = 41.74
        self.MAX_RAW_VEL = 250

        # 오도메트리 누적 위치
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = self.get_clock().now()

        # Control Table 주소 (Protocol 2.0 - MX-106)
        self.P2_ADDR_OPERATING_MODE = 11
        self.P2_ADDR_TORQUE_ENABLE  = 64
        self.P2_ADDR_HARDWARE_ERROR = 70
        self.P2_ADDR_PROFILE_VELOCITY= 112
        self.P2_ADDR_GOAL_POSITION   = 116
        self.P2_ADDR_GOAL_VELOCITY   = 104
        self.P2_ADDR_PRESENT_POS     = 132

        # Control Table 주소 (Protocol 1.0 - EX-106)
        self.P1_ADDR_RETURN_DELAY   = 5
        self.P1_ADDR_CW_ANGLE_LIMIT = 6
        self.P1_ADDR_CCW_ANGLE_LIMIT= 8
        self.P1_ADDR_TORQUE_ENABLE  = 24
        self.P1_ADDR_GOAL_POSITION  = 30
        self.P1_ADDR_MOVING_SPEED   = 32
        self.P1_ADDR_PRESENT_POS    = 36

        # Port & Packet Handler
        self.portHandler = PortHandler(self.DEVICENAME)
        self.p1_handler = PacketHandler(1.0)
        self.p2_handler = PacketHandler(2.0)

        # 복구 진행 중 중복 실행 방지 플래그
        self.is_rebooting_id5 = False

        # 포트 열기 및 보레이트 설정
        try:
            if not self.portHandler.openPort() or not self.portHandler.setBaudRate(self.BAUDRATE):
                self.get_logger().error("⚠️ 다이나믹셀 포트 열기/보레이트 설정 실패!")
                return
        except Exception as e:
            self.get_logger().error(f"⚠️ 시리얼 포트 접근 중 예외 발생: {e}")
            return

        self.check_all_motors_connection()
        self.init_all_motors()

        # ROS 2 Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.encoder_pub = self.create_publisher(JointState, 'motor_encoder_positions', 10)

        # ROS 2 Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10, callback_group=self.cb_group)
        
        self.motor_vel_sub = self.create_subscription(
            JointState, 'motor_cmd_vel', self.motor_cmd_vel_callback, 10, callback_group=self.cb_group)
        
        self.motor_pos_sub = self.create_subscription(
            JointState, 'motor_cmd_pos', self.motor_cmd_pos_callback, 10, callback_group=self.cb_group)

        # 주기적 엔코더 읽기 / 오도메트리 루프 (20Hz)
        self.timer = self.create_timer(0.05, self.update_loop, callback_group=self.cb_group)

    # =========================================================================
    # ★ 수정: 안전한 5번 모터 Reboot 및 복구 로직 ★
    # =========================================================================
    def reboot_5번_motor(self):
        """5번 MX-106 모터 소프트웨어 재부팅(Reboot) 및 안정적인 복구"""
        if self.is_rebooting_id5:
            return

        self.is_rebooting_id5 = True
        self.get_logger().warn("🔄 [5번 모터] 셧다운/응답불능 감지! 모터 소프트웨어 Reboot 시도 중...")

        # 비동기 또는 다른 스레드 접근을 방지하기 위해 별도 스레드로 복구 수행
        threading.Thread(target=self._reboot_5_process, daemon=True).start()

    def _reboot_5_process(self):
        try:
            with self.dxl_lock:
                # 1. Reboot 패킷 전송
                res, err = self.p2_handler.reboot(self.portHandler, 5)
                if res == COMM_SUCCESS:
                    self.get_logger().info("✅ [5번 모터] Reboot 패킷 전송 성공!")
                else:
                    self.get_logger().error(f"❌ [5번 모터] Reboot 패킷 전송 실패 (res: {res}, err: {err})")

            # 2. 부팅 대기 (Lock 해제 후 대기하여 다른 모터 패킷 차단 완화)
            time.sleep(1.0)

            # 3. Ping 테스트로 모터가 살아났는지 확인 (최대 5회 시도)
            motor_ready = False
            for attempt in range(5):
                with self.dxl_lock:
                    _, res, err = self.p2_handler.ping(self.portHandler, 5)
                    if res == COMM_SUCCESS and err == 0:
                        motor_ready = True
                        break
                time.sleep(0.3)

            if not motor_ready:
                self.get_logger().error("❌ [5번 모터] Reboot 후 Ping 응답 없음. 복구 실패.")
                return

            # 4. Multi-turn 모드 및 Torque 재설정
            with self.dxl_lock:
                self.get_logger().info("⚙️ [5번 모터] Multi-turn 모드 및 토크 재설정 중...")
                self.p2_handler.write1ByteTxRx(self.portHandler, 5, self.P2_ADDR_TORQUE_ENABLE, 0)
                self.p2_handler.write1ByteTxRx(self.portHandler, 5, self.P2_ADDR_OPERATING_MODE, 4)
                self.p2_handler.write1ByteTxRx(self.portHandler, 5, self.P2_ADDR_TORQUE_ENABLE, 1)

            self.get_logger().info("🎉 [5번 모터] 정상적으로 복구되었습니다!")
        except Exception as e:
            self.get_logger().error(f"❌ [5번 모터] 복구 처리 도중 예외 발생: {e}")
        finally:
            self.is_rebooting_id5 = False

    def handle_comm_error(self, target_id=None):
        """통신 에러 처리"""
        if target_id == 5:
            if not self.is_rebooting_id5:
                self.reboot_5번_motor()
        else:
            self.get_logger().warn(f"⚠️ 물리 통신 에러 감지 (ID: {target_id}): 시리얼 포트 재연결 시도 중...")
            try:
                self.portHandler.closePort()
                time.sleep(0.1)
                if self.portHandler.openPort() and self.portHandler.setBaudRate(self.BAUDRATE):
                    self.get_logger().info("✅ 포트 재연결 성공!")
                    self.stop_all_velocity_motors()
                else:
                    self.get_logger().error("❌ 포트 재연결 실패.")
            except Exception as e:
                self.get_logger().error(f"포트 복구 중 예외 발생: {e}")

    def safe_write(self, func, *args):
        """쓰기 명령 수행 안전 래퍼"""
        m_id = args[1] if len(args) > 1 else None
        # 5번 모터 재부팅 중일 땐 5번 쓰기 동작 무시
        if m_id == 5 and self.is_rebooting_id5:
            return COMM_NOT_AVAILABLE, 0

        res, err = func(*args)
        if res != COMM_SUCCESS or err != 0:
            self.handle_comm_error(target_id=m_id)
        return res, err

    def safe_read(self, func, *args):
        """읽기 명령 수행 안전 래퍼"""
        m_id = args[1] if len(args) > 1 else None
        if m_id == 5 and self.is_rebooting_id5:
            return None

        data, res, err = func(*args)
        if res != COMM_SUCCESS or err != 0:
            self.handle_comm_error(target_id=m_id)
            return None
        return data

    # =========================================================================
    # 기존 모터 초기화 / 제어 함수들
    # =========================================================================
    def check_all_motors_connection(self):
        self.get_logger().info("🔍 모터 통신 상태 진단 시작...")
        connected_ids, failed_ids = [], []

        with self.dxl_lock:
            for m_id in self.ALL_IDS:
                try:
                    if m_id in self.MX_VEL_IDS or m_id in self.MX_EXT_POS_IDS:
                        _, res, err = self.p2_handler.ping(self.portHandler, m_id)
                    else:
                        _, res, err = self.p1_handler.ping(self.portHandler, m_id)

                    if res == COMM_SUCCESS and err == 0:
                        connected_ids.append(m_id)
                    else:
                        failed_ids.append(m_id)
                except Exception as e:
                    failed_ids.append(m_id)

        print("\n" + "="*50)
        print(f"    [모터 연결 진단 결과]")
        print(f"    ✅ 연결 성공 ({len(connected_ids)}개): {connected_ids}")
        if failed_ids:
            print(f"    ❌ 연결 실패 ({len(failed_ids)}개): {failed_ids}")
        print("="*50 + "\n")

    def init_all_motors(self):
        with self.dxl_lock:
            try:
                for m_id in self.MX_VEL_IDS:
                    self.p2_handler.write1ByteTxRx(self.portHandler, m_id, self.P2_ADDR_TORQUE_ENABLE, 0)
                    self.p2_handler.write1ByteTxRx(self.portHandler, m_id, self.P2_ADDR_OPERATING_MODE, 1)
                    self.p2_handler.write1ByteTxRx(self.portHandler, m_id, self.P2_ADDR_TORQUE_ENABLE, 1)

                for m_id in self.MX_EXT_POS_IDS:
                    self.p2_handler.write1ByteTxRx(self.portHandler, m_id, self.P2_ADDR_TORQUE_ENABLE, 0)
                    self.p2_handler.write1ByteTxRx(self.portHandler, m_id, self.P2_ADDR_OPERATING_MODE, 4)
                    self.p2_handler.write1ByteTxRx(self.portHandler, m_id, self.P2_ADDR_TORQUE_ENABLE, 1)

                for m_id in self.EX_VEL_IDS + self.EX_POS_IDS:
                    self.p1_handler.write1ByteTxRx(self.portHandler, m_id, self.P1_ADDR_RETURN_DELAY, 0)

                for m_id in self.EX_VEL_IDS:
                    self.p1_handler.write1ByteTxRx(self.portHandler, m_id, self.P1_ADDR_TORQUE_ENABLE, 0)
                    self.p1_handler.write2ByteTxRx(self.portHandler, m_id, self.P1_ADDR_CW_ANGLE_LIMIT, 0)
                    self.p1_handler.write2ByteTxRx(self.portHandler, m_id, self.P1_ADDR_CCW_ANGLE_LIMIT, 0)
                    self.p1_handler.write1ByteTxRx(self.portHandler, m_id, self.P1_ADDR_TORQUE_ENABLE, 1)

                for m_id in self.EX_POS_IDS:
                    self.p1_handler.write1ByteTxRx(self.portHandler, m_id, self.P1_ADDR_TORQUE_ENABLE, 0)
                    self.p1_handler.write2ByteTxRx(self.portHandler, m_id, self.P1_ADDR_CW_ANGLE_LIMIT, 0)
                    self.p1_handler.write2ByteTxRx(self.portHandler, m_id, self.P1_ADDR_CCW_ANGLE_LIMIT, 4095)
                    self.p1_handler.write1ByteTxRx(self.portHandler, m_id, self.P1_ADDR_TORQUE_ENABLE, 1)

                self.get_logger().info("✅ 모든 모터 초기화 완료")
            except Exception as e:
                self.get_logger().error(f"모터 초기화 중 예외 발생: {e}")

    def stop_all_velocity_motors(self):
        try:
            for m_id in self.MX_VEL_IDS:
                self.p2_handler.write4ByteTxRx(self.portHandler, m_id, self.P2_ADDR_GOAL_VELOCITY, 0)
            for m_id in self.EX_VEL_IDS:
                self.p1_handler.write2ByteTxRx(self.portHandler, m_id, self.P1_ADDR_MOVING_SPEED, 0)
        except Exception as e:
            pass

    def get_mx_position(self, m_id):
        with self.dxl_lock:
            pos = self.safe_read(self.p2_handler.read4ByteTxRx, self.portHandler, m_id, self.P2_ADDR_PRESENT_POS)
            if pos is None:
                return None
            if pos > 0x7FFFFFFF:
                pos -= 0x100000000
            return pos

    def get_ex_position(self, m_id):
        with self.dxl_lock:
            return self.safe_read(self.p1_handler.read2ByteTxRx, self.portHandler, m_id, self.P1_ADDR_PRESENT_POS)

    def cmd_vel_callback(self, msg):
        try:
            v, w = msg.linear.x, msg.angular.z
            left_speed = (v - (w * self.wheel_separation / 2.0)) / self.wheel_radius
            right_speed = (v + (w * self.wheel_separation / 2.0)) / self.wheel_radius

            l_val = max(min(int(left_speed * self.VELOCITY_CONSTANT), self.MAX_RAW_VEL), -self.MAX_RAW_VEL)
            r_val = max(min(int(right_speed * self.VELOCITY_CONSTANT), self.MAX_RAW_VEL), -self.MAX_RAW_VEL)

            with self.dxl_lock:
                self.safe_write(self.p2_handler.write4ByteTxRx, self.portHandler, 1, self.P2_ADDR_GOAL_VELOCITY, l_val & 0xFFFFFFFF)
                self.safe_write(self.p2_handler.write4ByteTxRx, self.portHandler, 2, self.P2_ADDR_GOAL_VELOCITY, -r_val & 0xFFFFFFFF)
        except Exception as e:
            pass

    def motor_cmd_vel_callback(self, msg):
        try:
            for name, vel in zip(msg.name, msg.velocity):
                m_id = int(name.split('_')[-1])
                speed = int(vel)
                with self.dxl_lock:
                    if m_id in self.EX_VEL_IDS:
                        val = int(abs(speed))
                        if speed < 0:
                            val += 1024
                        val = min(val, 2047) if speed < 0 else min(val, 1023)
                        self.safe_write(self.p1_handler.write2ByteTxRx, self.portHandler, m_id, self.P1_ADDR_MOVING_SPEED, val)
        except Exception as e:
            pass

    def motor_cmd_pos_callback(self, msg):
        try:
            for idx, name in enumerate(msg.name):
                m_id = int(name.split('_')[-1])
                target_pos = int(msg.position[idx])
                has_vel = len(msg.velocity) > idx
                target_vel = int(msg.velocity[idx]) if has_vel else None

                with self.dxl_lock:
                    if m_id in self.MX_EXT_POS_IDS:
                        if self.is_rebooting_id5:
                            continue
                        target_pos = max(min(target_pos, 1048575), -1048575)
                        if target_vel is not None and target_vel > 0:
                            self.safe_write(self.p2_handler.write4ByteTxRx, self.portHandler, m_id, self.P2_ADDR_PROFILE_VELOCITY, target_vel)
                        self.safe_write(self.p2_handler.write4ByteTxRx, self.portHandler, m_id, self.P2_ADDR_GOAL_POSITION, target_pos & 0xFFFFFFFF)

                    elif m_id in self.EX_POS_IDS:
                        target_pos = max(min(target_pos, 4095), 0)
                        ex_speed = 200 if m_id != 7 else 0
                        if target_vel is not None:
                            ex_speed = 0 if target_vel <= 0 else max(1, min(target_vel, 1023))

                        self.safe_write(self.p1_handler.write2ByteTxRx, self.portHandler, m_id, self.P1_ADDR_MOVING_SPEED, ex_speed)
                        self.safe_write(self.p1_handler.write2ByteTxRx, self.portHandler, m_id, self.P1_ADDR_GOAL_POSITION, target_pos)
        except Exception as e:
            pass

    def update_loop(self):
        try:
            current_time = self.get_clock().now()
            if not hasattr(self, 'last_time') or self.last_time is None:
                self.last_time = current_time
                return

            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # 1. 오도메트리 계산
            if dt > 0:
                l_ticks = self.get_mx_position(1)
                r_ticks = self.get_mx_position(2)

                if l_ticks is not None and r_ticks is not None:
                    if self.last_left_ticks is None or self.last_right_ticks is None:
                        self.last_left_ticks = l_ticks
                        self.last_right_ticks = r_ticks
                    else:
                        d_l = (l_ticks - self.last_left_ticks) / self.TICKS_PER_REV * (2 * math.pi * self.wheel_radius)
                        d_r = -(r_ticks - self.last_right_ticks) / self.TICKS_PER_REV * (2 * math.pi * self.wheel_radius)

                        self.last_left_ticks = l_ticks
                        self.last_right_ticks = r_ticks

                        d_center = (d_l + d_r) / 2.0
                        d_th = (d_r - d_l) / self.wheel_separation

                        self.x += d_center * math.cos(self.th + d_th / 2.0)
                        self.y += d_center * math.sin(self.th + d_th / 2.0)
                        self.th += d_th

                        v_x = d_center / dt
                        v_th = d_th / dt

                        qz = math.sin(self.th / 2.0)
                        qw = math.cos(self.th / 2.0)

                        odom = Odometry()
                        odom.header.stamp = current_time.to_msg()
                        odom.header.frame_id = 'odom'
                        odom.child_frame_id = 'base_footprint'
                        odom.pose.pose.position.x = self.x
                        odom.pose.pose.position.y = self.y
                        odom.pose.pose.orientation.z = qz
                        odom.pose.pose.orientation.w = qw
                        odom.twist.twist.linear.x = v_x
                        odom.twist.twist.angular.z = v_th
                        self.odom_pub.publish(odom)

                        js = JointState()
                        js.header.stamp = current_time.to_msg()
                        js.name = ['left_wheel_joint', 'right_wheel_joint']
                        js.position = [(l_ticks / self.TICKS_PER_REV) * 2 * math.pi, -(r_ticks / self.TICKS_PER_REV) * 2 * math.pi]
                        self.joint_pub.publish(js)

            # 2. 엔코더 데이터 퍼블리시
            enc_msg = JointState()
            enc_msg.header.stamp = current_time.to_msg()

            for m_id in self.MX_VEL_IDS + self.MX_EXT_POS_IDS:
                pos = self.get_mx_position(m_id)
                if pos is not None:
                    enc_msg.name.append(f'motor_{m_id}')
                    enc_msg.position.append(float(pos))

            for m_id in self.EX_VEL_IDS:
                pos = self.get_ex_position(m_id)
                if pos is not None:
                    enc_msg.name.append(f'motor_{m_id}')
                    enc_msg.position.append(float(pos))

            if enc_msg.name:
                self.encoder_pub.publish(enc_msg)
        except Exception as e:
            pass

    def destroy_node(self):
        self.get_logger().info("🛑 노드 종료 감지")
        try:
            self.stop_all_velocity_motors()
            time.sleep(0.05)
            with self.dxl_lock:
                if hasattr(self, 'portHandler') and self.portHandler is not None:
                    self.portHandler.closePort()
        except Exception as e:
            pass
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MixedMotorControlNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        def sig_handler(sig, frame):
            if node is not None:
                node.destroy_node()
            sys.exit(0)

        signal.signal(signal.SIGINT, sig_handler)
        signal.signal(signal.SIGTERM, sig_handler)

        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"메인 실행 중 치명적 예외 발생: {e}")
    finally:
        if node is not None and rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
