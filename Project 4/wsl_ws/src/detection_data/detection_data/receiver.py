import socket
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def get_windows_host_ip():
    return "127.0.0.1"


class ReceiverNode(Node):
    def __init__(self):
        super().__init__("receiver_node")
        self.pub = self.create_publisher(String, "/hello", 10)
        host = get_windows_host_ip()
        self.get_logger().info(f"Connecting to {host}:5005 ...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, 5005))
        self.get_logger().info("Connected!")
        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _recv_loop(self):
        buf = ""
        while self._running:
            chunk = self.sock.recv(1024).decode("utf-8", errors="ignore")
            if not chunk:
                break
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                if line.strip():
                    msg = String()
                    msg.data = line.strip()
                    self.pub.publish(msg)
                    self.get_logger().info(f"Received: {line.strip()}")

    def destroy_node(self):
        self._running = False
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
