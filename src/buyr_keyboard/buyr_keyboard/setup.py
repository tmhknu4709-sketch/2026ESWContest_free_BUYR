#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class KeyboardMinMaxPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_minmax_publisher')
        self.min_pub = self.create_publisher(Empty, 'min_topic', 10)
        self.max_pub = self.create_publisher(Empty, 'max_topic', 10)
        self.get_logger().info("A 입력 → /min_topic | B 입력 → /max_topic | q → 종료")

    def run(self):
        while rclpy.ok():
            key = input("입력(A/B/q): ").strip().upper()
            msg = Empty()

            if key == 'A':
                self.min_pub.publish(msg)
                self.get_logger().info("Published /min_topic")
            elif key == 'B':
                self.max_pub.publish(msg)
                self.get_logger().info("Published /max_topic")
            elif key == 'Q':
                self.get_logger().info("종료합니다.")
                break
            else:
                print("A 또는 B를 입력하세요.")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardMinMaxPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

