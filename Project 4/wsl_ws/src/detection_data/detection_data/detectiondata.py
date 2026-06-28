#!/usr/bin/env python3
"""
ROS2 node: zed_detection_subscriber
-------------------------------------
Connects to the ZED detection TCP server (default 127.0.0.1:5005).

Behaviour
---------
- OBJECT lines  → published immediately every frame (live dims)
- FRAME lines   → buffered internally, NOT published yet
- SCAN_COMPLETE|1 received → publish the buffered FRAME summary once,
                              then clear the buffer

This ensures /zed_detection/frame_summary, /zed_detection/recommended_box,
/zed_detection/total_volume, and /zed_detection/object_count only fire
once per completed scan cycle.

Published topics
----------------
  /zed_detection/object          std_msgs/String   — JSON per object (live)
  /zed_detection/frame_summary   std_msgs/String   — JSON frame result (on scan complete)
  /zed_detection/recommended_box std_msgs/String   — Small / Medium / Large / None
  /zed_detection/total_volume    std_msgs/Float32  — total volume cm3
  /zed_detection/object_count    std_msgs/Int32    — number of objects
  /zed_detection/scan_complete   std_msgs/Int32    — publishes 1 on scan complete

ROS2 parameters
---------------
  tcp_host        (string)  default: "127.0.0.1"
  tcp_port        (int)     default: 5005
  reconnect_delay (float)   default: 3.0

Usage
-----
  ros2 run detectionData detectiondata
  ros2 run detectionData detectiondata --ros-args -p tcp_host:=192.168.1.10
"""

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String


# ── line parsers ──────────────────────────────────────────────────────────────

def parse_kv(segment: str) -> dict:
    """Parse a comma-separated key:value segment into a plain dict."""
    result = {}
    for token in segment.strip().split(","):
        if ":" in token:
            k, _, v = token.partition(":")
            result[k.strip()] = v.strip()
    return result


def parse_object_line(line: str) -> dict | None:
    """
    Parse a per-object line.
    Format: OBJECT|l:12.3,w:8.5,h:6.0,vol:627.6
    """
    if not line.startswith("OBJECT|"):
        return None
    kv = parse_kv(line[len("OBJECT|"):])
    try:
        return {
            "l":   float(kv.get("l",   0)),
            "w":   float(kv.get("w",   0)),
            "h":   float(kv.get("h",   0)),
            "vol": float(kv.get("vol", 0)),
        }
    except ValueError:
        return None


def parse_frame_line(line: str) -> dict | None:
    """
    Parse a per-frame summary line.
    Format: FRAME|objs:3,total_vol:1882.8,recommended:Small
    """
    if not line.startswith("FRAME|"):
        return None
    kv = parse_kv(line[len("FRAME|"):])
    try:
        return {
            "objs":        int(kv.get("objs", 0)),
            "total_vol":   float(kv.get("total_vol", 0)),
            "recommended": kv.get("recommended", "None"),
        }
    except ValueError:
        return None


def parse_scan_complete(line: str) -> bool:
    """
    Detect the SCAN_COMPLETE signal.
    Format: SCAN_COMPLETE|1
    """
    return line.strip() == "SCAN_COMPLETE|1"


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class ZedDetectionSubscriber(Node):

    def __init__(self):
        super().__init__("zed_detection_subscriber")

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter("tcp_host",        "127.0.0.1") #beckh's tcp host: 127.0.0.1; 	Aloy:172.20.10.2
        self.declare_parameter("tcp_port",        5005)
        self.declare_parameter("reconnect_delay", 3.0)

        self.tcp_host        = self.get_parameter("tcp_host").value
        self.tcp_port        = self.get_parameter("tcp_port").value
        self.reconnect_delay = self.get_parameter("reconnect_delay").value

        # ── publishers ────────────────────────────────────────────────────────
        self.pub_object        = self.create_publisher(
            String,  "/zed_detection/object",          10)
        self.pub_frame         = self.create_publisher(
            String,  "/zed_detection/frame_summary",   10)
        self.pub_recommended   = self.create_publisher(
            String,  "/zed_detection/recommended_box", 10)
        self.pub_total_vol     = self.create_publisher(
            Float32, "/zed_detection/total_volume",    10)
        self.pub_obj_count     = self.create_publisher(
            Int32,   "/zed_detection/object_count",    10)
        self.pub_scan_complete = self.create_publisher(
            Int32,   "/zed_detection/scan_complete",   10)

        # ── internal state ────────────────────────────────────────────────────
        # Holds the latest FRAME data received during the scan window.
        # Only published when SCAN_COMPLETE|1 arrives.
        self._pending_frame: dict | None = None
        self._state_lock = threading.Lock()

        # ── TCP receiver thread ───────────────────────────────────────────────
        self._running = True
        self._thread  = threading.Thread(target=self._tcp_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"zed_detection_subscriber started — "
            f"connecting to {self.tcp_host}:{self.tcp_port}"
        )

    # ── TCP loop ──────────────────────────────────────────────────────────────

    def _tcp_loop(self):
        while self._running:
            sock = None
            try:
                self.get_logger().info(
                    f"Connecting to {self.tcp_host}:{self.tcp_port} ..."
                )
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self.tcp_host, self.tcp_port))
                sock.settimeout(None)
                self.get_logger().info("TCP connection established.")

                buf = ""
                while self._running:
                    chunk = sock.recv(4096)
                    if not chunk:
                        self.get_logger().warn("Server closed the connection.")
                        break
                    buf += chunk.decode(errors="replace")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        self._handle_line(line.strip())

            except (ConnectionRefusedError, OSError) as exc:
                self.get_logger().warn(
                    f"TCP error: {exc} — retrying in {self.reconnect_delay}s"
                )
            finally:
                if sock:
                    try:
                        sock.close()
                    except OSError:
                        pass

            if self._running:
                time.sleep(self.reconnect_delay)

    # ── dispatcher ────────────────────────────────────────────────────────────

    def _handle_line(self, line: str):
        if not line:
            return

        # 1. SCAN_COMPLETE signal → flush pending frame data
        if parse_scan_complete(line):
            self._on_scan_complete()
            return

        # 2. Per-object → publish immediately
        obj_data = parse_object_line(line)
        if obj_data is not None:
            self._publish_object(obj_data)
            return

        # 3. Frame summary → buffer it, do NOT publish yet
        frame_data = parse_frame_line(line)
        if frame_data is not None:
            with self._state_lock:
                self._pending_frame = frame_data
            return

        self.get_logger().debug(f"Unrecognised line: {line}")

    # ── scan complete handler ─────────────────────────────────────────────────

    def _on_scan_complete(self):
        """Called when SCAN_COMPLETE|1 is received. Publishes buffered frame."""

        # Publish scan_complete flag
        sc_msg      = Int32()
        sc_msg.data = 1
        self.pub_scan_complete.publish(sc_msg)
        self.get_logger().info("SCAN_COMPLETE received — publishing frame summary.")

        with self._state_lock:
            data = self._pending_frame
            self._pending_frame = None   # clear buffer

        if data is None:
            self.get_logger().warn("SCAN_COMPLETE received but no frame data buffered.")
            return

        self._publish_frame(data)

    # ── per-object publisher ──────────────────────────────────────────────────

    def _publish_object(self, data: dict):
        msg      = String()
        msg.data = json.dumps(data)
        self.pub_object.publish(msg)

        self.get_logger().info(
            f"OBJECT  "
            f"L:{data['l']:.1f}cm  "
            f"W:{data['w']:.1f}cm  "
            f"H:{data['h']:.1f}cm  "
            f"Vol:{data['vol']:.1f}cm3"
        )

    # ── per-frame publisher ───────────────────────────────────────────────────

    def _publish_frame(self, data: dict):
        # full JSON summary
        frame_msg      = String()
        frame_msg.data = json.dumps(data)
        self.pub_frame.publish(frame_msg)

        # recommended box name
        rec_msg      = String()
        rec_msg.data = data["recommended"]
        self.pub_recommended.publish(rec_msg)

        # total volume cm3
        vol_msg      = Float32()
        vol_msg.data = float(data["total_vol"])
        self.pub_total_vol.publish(vol_msg)

        # object count
        cnt_msg      = Int32()
        cnt_msg.data = int(data["objs"])
        self.pub_obj_count.publish(cnt_msg)

        self.get_logger().info(
            f"FRAME   "
            f"objs:{data['objs']}  "
            f"total_vol:{data['total_vol']:.1f}cm3  "
            f"recommended:{data['recommended']}"
        )

    # ── shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        super().destroy_node()


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ZedDetectionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
