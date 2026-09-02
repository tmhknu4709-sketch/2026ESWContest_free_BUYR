import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_class_and_detect_controller')

        self.class_publisher = self.create_publisher(String, '/set_target_class', 10)
        self.detect_publisher = self.create_publisher(String, '/detect_now', 10)

        self.get_logger().info("🎹 KeyControlNode started.")
        self.get_logger().info("Press keys directly (no Enter needed)")
        self.get_logger().info("e=egg c=coka m=mandarin s=strawberry d=detect q=quit")

        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            if key == 'e':
                self.send_class('egg')
            elif key == 'c':
                self.send_class('coka_cap')
            elif key == 'm':
                self.send_class('mandarin')
            elif key == 's':
                self.send_class('strawberry')
            elif key == 'd':
                self.trigger_detection()
            elif key == 'q':
                self.get_logger().info("🛑 Quit")
                break

    def send_class(self, class_name):
        msg = String()
        msg.data = class_name
        self.class_publisher.publish(msg)
        self.get_logger().info(f"🎯 Target: {class_name}")

    def trigger_detection(self):
        msg = String()
        msg.data = "detect"
        self.detect_publisher.publish(msg)
        self.get_logger().info("🚀 Detection triggered")


def main(args=None):
    rclpy.init(args=args)
    node = KeyControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

