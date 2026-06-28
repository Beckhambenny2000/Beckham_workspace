import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import tty
import termios


class ZedDetectionPublisher(Node):

    def __init__(self):
        super().__init__('zed_detection_publisher')

        self.publisher = self.create_publisher(
            String,
            '/zed_detection/recommended_box',
            10
        )

        self.get_logger().info("ZED Detection Publisher ready.")
        self.get_logger().info("Press 'S' = Small | 'M' = Medium | 'L' = Large | 'q' = Quit")

    def publish_box(self, box_name: str):
        msg = String()
        msg.data = box_name
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: '{box_name}'")


def get_key():
    """Read a single keypress without requiring Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def keyboard_thread(node: ZedDetectionPublisher):
    key_map = {
        's': 'Small',
        'm': 'Medium',
        'l': 'Large',
    }

    while rclpy.ok():
        key = get_key()

        if key == 'q':
            node.get_logger().info("Quitting...")
            rclpy.shutdown()
            break

        elif key in key_map:
            node.publish_box(key_map[key])

        else:
            node.get_logger().warn(f"Unknown key '{key}'. Use S / M / L / q.")


def main(args=None):
    rclpy.init(args=args)

    node = ZedDetectionPublisher()

    kb_thread = threading.Thread(
        target=keyboard_thread,
        args=(node,),
        daemon=True
    )
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
