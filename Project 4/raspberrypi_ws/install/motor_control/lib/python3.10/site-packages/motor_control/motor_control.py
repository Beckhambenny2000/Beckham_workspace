#!/usr/bin/env python3
"""
ROS2 node: servo_controller
-----------------------------
Subscribes to /zed_detection/recommended_box (std_msgs/String)
and drives the matching servo motor via hardware PWM on a Raspberry Pi.

Box → GPIO pin mapping
----------------------
  Small  → GPIO 12  (PIN_MAP 'S')
  Medium → GPIO 13  (PIN_MAP 'M')
  Large  → GPIO 18  (PIN_MAP 'L')

Servo sequence
-------------------------------------------------
  1. Move to 0°   — 1 s
  2. Move to 60°  — 1 s
  3. Return to 0° — 1 s
  4. Cut signal (0) — stop jitter
"""

import threading
import time

import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── Pin mapping (Hardware PWM pins) ──────────────────────────────────────────
PIN_MAP = {
    'S': 12,   # Small  - GPIO 12
    'M': 13,   # Medium - GPIO 13
    'L': 18    # Large  - GPIO 18
}

# ── Box name → PIN_MAP key ────────────────────────────────────────────────────
BOX_KEY_MAP = {
    "Small":  "S",
    "Medium": "M",
    "Large":  "L",
}

FREQUENCY = 50
MIN_DUTY  = 2.5    # 0°
MAX_DUTY  = 12.5   # 180°

# ── Duty cycle for 60° ────────────────────────────────────────────────────────
# Formula: MIN_DUTY + (angle / 180) * (MAX_DUTY - MIN_DUTY)
SIXTY_DEG_DUTY = MIN_DUTY + (60 / 180) * (MAX_DUTY - MIN_DUTY)  # ≈ 5.83


# ── Servo helpers ─────────────────────────────────────────────────────────────

def setup():
    GPIO.setmode(GPIO.BCM)
    pwms = {}
    for pin in PIN_MAP.values():
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, FREQUENCY)
        pwm.start(0)  # Start with signal OFF
        pwms[pin] = pwm
    return pwms


def servo_move_and_stop(key, pwm):
    pin = PIN_MAP[key]
    print(f"[{key}] PIN {pin}: Moving to 0° (home position)...")
    pwm.ChangeDutyCycle(MIN_DUTY)       # Move to 0°
    time.sleep(1)

    print(f"[{key}] PIN {pin}: Moving to 60°...")
    pwm.ChangeDutyCycle(SIXTY_DEG_DUTY) # Move to 60°
    time.sleep(1)

    print(f"[{key}] PIN {pin}: Returning to 0°...")
    pwm.ChangeDutyCycle(MIN_DUTY)       # Return to 0°
    time.sleep(1)

    pwm.ChangeDutyCycle(0)              # Cut signal — stop jitter
    print(f"[{key}] PIN {pin}: Done.\n")


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class ServoController(Node):

    def __init__(self):
        super().__init__("servo_controller")

        # GPIO + PWM setup
        self._pwms = setup()
        self.get_logger().info(
            "GPIO initialised — "
            "S→GPIO12 (Small)  M→GPIO13 (Medium)  L→GPIO18 (Large)"
        )
        self.get_logger().info(
            f"Servo motion: 0° → 60° → 0°  "
            f"(duty cycles: {MIN_DUTY} → {SIXTY_DEG_DUTY:.2f} → {MIN_DUTY})"
        )

        # Servo runs in a background thread so it never blocks the ROS2 spin
        self._servo_lock = threading.Lock()
        self._servo_busy = False

        # Subscriber
        self._sub = self.create_subscription(
            String,
            "/zed_detection/recommended_box",
            self._recommended_box_callback,
            10,
        )

        self.get_logger().info(
            "Subscribed to /zed_detection/recommended_box"
        )

    # ── callback ──────────────────────────────────────────────────────────────

    def _recommended_box_callback(self, msg: String):
        box_name = msg.data.strip()

        if box_name == "None" or box_name not in BOX_KEY_MAP:
            self.get_logger().debug(
                f"Ignoring recommendation: '{box_name}'"
            )
            return

        # Only skip if servo is physically still moving from previous cycle
        with self._servo_lock:
            if self._servo_busy:
                self.get_logger().warn(
                    f"Servo busy — skipping '{box_name}'"
                )
                return
            self._servo_busy = True

        key = BOX_KEY_MAP[box_name]
        self.get_logger().info(
            f"Recommended: {box_name} → actuating servo key '{key}' "
            f"on GPIO {PIN_MAP[key]}"
        )

        # Run servo motion in background thread
        t = threading.Thread(
            target=self._run_servo,
            args=(key,),
            daemon=True,
        )
        t.start()

    # ── servo worker ──────────────────────────────────────────────────────────

    def _run_servo(self, key: str):
        try:
            servo_move_and_stop(key, self._pwms[PIN_MAP[key]])
        except Exception as exc:
            self.get_logger().error(f"Servo error: {exc}")
        finally:
            with self._servo_lock:
                self._servo_busy = False

    # ── shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info("Shutting down — cleaning up GPIO.")
        for pwm in self._pwms.values():
            pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
