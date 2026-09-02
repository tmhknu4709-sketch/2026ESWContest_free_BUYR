#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from pynput import keyboard


class KeyboardMinMaxPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_minmax_publisher')

        # 기존 토픽
        self.min_pub = self.create_publisher(Int32, 'min_topic', 10)
        self.max_pub = self.create_publisher(Int32, 'max_topic', 10)

        # 추가된 토픽
        self.front_pub = self.create_publisher(Bool, 'front_move', 10)
        self.back_pub = self.create_publisher(Bool, 'back_move', 10)
        self.stop_pub = self.create_publisher(Bool, 'stop_move', 10)  # ★ 추가된 stop 토픽

        self.get_logger().info(
            "A → /max_topic | S → /min_topic | Z → /front_move | "
            "X → /back_move | C → /stop_move | Q → 종료"
        )

        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def on_key_press(self, key):
        try:
            char = key.char.upper()
        except AttributeError:
            return  # 특수키 무시

        # ---- 기존 기능 ----
        if char == 'S':
            msg = Int32()
            msg.data = 0
            self.min_pub.publish(msg)
            self.get_logger().info("S 입력 → /min_topic (0) 퍼블리시")

        elif char == 'A':
            msg = Int32()
            msg.data = 1
            self.max_pub.publish(msg)
            self.get_logger().info("A 입력 → /max_topic (1) 퍼블리시")

        # ---- front/back ----
        elif char == 'Z':
            msg = Bool()
            msg.data = True
            self.front_pub.publish(msg)
            self.get_logger().info("Z 입력 → /front_move (True) 퍼블리시")

        elif char == 'X':
            msg = Bool()
            msg.data = True
            self.back_pub.publish(msg)
            self.get_logger().info("X 입력 → /back_move (True) 퍼블리시")

        # ---- ★ STOP 기능 추가 ----
        elif char == 'C':
            msg = Bool()
            msg.data = True
            self.stop_pub.publish(msg)
            self.get_logger().info("C 입력 → /stop_move (True) 퍼블리시")

        # ---- 종료 ----
        elif char == 'Q':
            self.get_logger().info("Q 입력 → 종료합니다.")
            rclpy.shutdown()

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardMinMaxPublisher()
    node.run()
    node.destroy_node()


if __name__ == '__main__':
    main()
