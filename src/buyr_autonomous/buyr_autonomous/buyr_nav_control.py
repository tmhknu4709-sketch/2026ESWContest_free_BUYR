import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math
import time
import numpy as np
import sys
import termios
import tty

class Nav2FullPlanner(Node):
    def __init__(self):
        super().__init__('nav2_full_planner')
        
        # 1. Pub/Sub 및 Action Client 설정
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        self.latest_scan = None
        
        # 2. 좌표 데이터 (경유지 포함)
        self.goals = {
            '1': {'x': -0.0397418, 'y': 0.159425, 'qz': 0.132385, 'qw': 0.991198},
            '2': {'x': 1.32102, 'y': 0.796156, 'qz': 0.123815, 'qw': 0.992305},
            'waypoint': {
                'x': -0.0326567, 
                'y': 0.764216, 
                'qz': 0.121776, 
                'qw': 0.992558
            }
        }
        
        self.get_logger().info("🤖 경유지 포함 정밀 내비게이터 활성화")
        self.get_logger().info("1: 원점 | 2: 경유 후 목적지+15cm정렬 | Ctrl+C: 종료")

    def scan_callback(self, msg):
        self.latest_scan = msg

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def move_distance(self, distance=0.1, speed=0.05):
        self.get_logger().info(f"📏 추가 이동: {distance*100}cm 전진")
        duration = abs(distance / speed)
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            if not rclpy.ok(): break
            cmd = Twist()
            cmd.linear.x = speed if distance > 0 else -speed
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        self.stop_robot()

    def get_table_distance(self):
        for _ in range(5): rclpy.spin_once(self, timeout_sec=0.01)
        if self.latest_scan is None: return None, None
        
        msg = self.latest_scan
        points_x, points_y = [], []
        search_angle = math.radians(20)
        
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if -search_angle < angle < search_angle and msg.range_min < r < msg.range_max:
                points_x.append(r * math.cos(angle))
                points_y.append(r * math.sin(angle))
        
        if len(points_x) < 10: return None, None
        
        A = np.vstack([points_y, np.ones(len(points_y))]).T
        m, c = np.linalg.lstsq(A, points_x, rcond=None)[0]
        return c, math.atan(m)

    def align_and_dock(self, target_dist=0.15):
        self.get_logger().info(f"📐 정렬 시작 (목표: {target_dist}m)")
        while rclpy.ok():
            dist, angle = self.get_table_distance()
            if dist is None: break
                
            cmd = Twist()
            if abs(angle) > math.radians(1.0):
                cmd.angular.z = -angle * 0.8
            else:
                cmd.angular.z = 0.0
                
            dist_err = dist - target_dist
            if abs(angle) < math.radians(3.0):
                if abs(dist_err) > 0.01:
                    speed = dist_err * 0.5
                    cmd.linear.x = max(min(speed, 0.05), 0.02) if dist_err > 0 else min(max(speed, -0.05), -0.02)
                else:
                    cmd.linear.x = 0.0

            if abs(dist_err) <= 0.01 and abs(angle) <= math.radians(1.0):
                self.stop_robot()
                self.get_logger().info("✅ 정렬 완료")
                break
                
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

    def execute_nav_to(self, coords):
        """실제 Nav2 Goal을 전송하고 완료될 때까지 기다리는 함수"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 서버 응답 없음')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = coords['x']
        goal_msg.pose.pose.position.y = coords['y']
        goal_msg.pose.pose.orientation.z = coords['qz']
        goal_msg.pose.pose.orientation.w = coords['qw']

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def send_goal(self, key):
        if key == '2':
            # 1. 경유지(Waypoint) 이동
            self.get_logger().info("🔄 1단계: 경유지로 이동합니다...")
            if self.execute_nav_to(self.goals['waypoint']):
                self.get_logger().info("📍 경유지 도착. 2단계: 최종 목적지로 이동합니다.")
                
                # 2. 최종 목적지 이동
                if self.execute_nav_to(self.goals['2']):
                    self.get_logger().info("🏁 목적지 도착. 정렬을 시작합니다.")
                    time.sleep(0.5)
                    # 3. 라이다 기반 15cm 정렬 (요청사항 반영)
                    self.align_and_dock(0.15) 
                    
                    # 4. 정렬 후 추가 진입이 필요 없다면 아래 줄은 주석 처리하거나 값을 조절하세요.
                    self.move_distance(0.1, 0.05) 
            else:
                self.get_logger().error("❌ 경유지 이동 실패")
        
        elif key == '1':
            self.get_logger().info("🏠 원점(1번)으로 이동합니다.")
            self.execute_nav_to(self.goals['1'])

def main(args=None):
    rclpy.init(args=args)
    node = Nav2FullPlanner()
    settings = termios.tcgetattr(sys.stdin)

    def get_key():
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    try:
        while True:
            key = get_key()
            if key in ['1', '2']:
                node.send_goal(key)
            elif key == '\x03': break
    except Exception as e:
        print(e)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
