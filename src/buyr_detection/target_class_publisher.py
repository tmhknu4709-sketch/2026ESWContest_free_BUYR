import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


class TargetClassPublisher(Node):
    def __init__(self):
        super().__init__('target_class_publisher')
        self.publisher = self.create_publisher(String, '/set_target_class', 10)
        self.get_logger().info("✅ TargetClassPublisher started. Publishing to '/set_target_class'")

    def publish_target(self, class_name: str):
        msg = String()
        msg.data = class_name
        self.publisher.publish(msg)
        self.get_logger().info(f"🎯 Sent target_class='{class_name}'")


def main(args=None):
    rclpy.init(args=args)
    node = TargetClassPublisher()

    # 명령행 인자 또는 입력으로 target_class 지정
    if len(sys.argv) > 1:
        class_name = sys.argv[1]
    else:
        class_name = input("Enter target class name (e.g., 'egg', 'coka', 'mandarin', 'strawberry'): ")

    node.publish_target(class_name)
    rclpy.spin_once(node, timeout_sec=1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

