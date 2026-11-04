#!/usr/bin/env python3
"""
keyboard_clutch.py - a global teleoperation enable on the PC keyboard.

    SPACE  toggle engaged / released        0  release        R  reset mock arms        q  quit

Publishes std_msgs/Float32 on /quest/clutch at 20 Hz (1.0 = engaged, 0.0 = released) and publishes 0.0
when it exits. Use it instead of the pinch gesture: pinching with the hand that also drives an arm and
fingers makes tracking worse and ties the clutch to that hand's tracking quality.

Run (workspace sourced, from a real terminal):   python3 ~/Vr-robot-teleop/tools/keyboard_clutch.py

The gesture clutch must stop publishing to the same topic: set `clutch_topic: /quest/clutch_gesture`
for every hand_inputs node (quest_hand/config/hand.yaml and your left hand_inputs). Then
`ros2 topic info /quest/clutch` should list exactly ONE publisher.
"""

import subprocess
from pathlib import Path

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ServoStatus
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy


class KeyboardClutch(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_clutch")
        self.declare_parameter("topic", "/quest/clutch")
        self.declare_parameter("rate", 20.0)
        self._pub = self.create_publisher(Float32, self.get_parameter("topic").value, 10)
        self.engaged = False
        self._arm_engaged = {}
        self._servo = {}
        self._pose_at = {}
        self._last_status = None
        for side, prefix in (("right", ""), ("left", "/left")):
            self.create_subscription(Bool, prefix + "/teleop/engaged",
                lambda m, s=side: self._arm_engaged.update({s: m.data}),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(ServoStatus, prefix + "/servo_node/status",
                lambda m, s=side: self._servo.update({s: m.message}), qos_profile_sensor_data)
            self.create_subscription(PoseStamped, f"/quest/{side}_hand/pose_raw",
                lambda m, s=side: self._pose_at.update({s: self.get_clock().now().nanoseconds}), qos_profile_sensor_data)
        self.create_timer(0.5, self.report_status)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.publish)

    def report_status(self) -> None:
        now = self.get_clock().now().nanoseconds
        parts = []
        for side in ("right", "left"):
            fresh = side in self._pose_at and (now - self._pose_at[side]) * 1e-9 <= 0.25
            engaged = self._arm_engaged.get(side)
            state = "unknown" if engaged is None else "engaged" if engaged else "released"
            parts.append(f"{side}: {state}, wrist {'fresh' if fresh else 'missing/stale'}, "
                         f"Servo: {self._servo.get(side, 'unknown')}")
        status = " | ".join(parts)
        if status != self._last_status:
            sys.stdout.write("\r\n" + status + "\r\n")
            sys.stdout.flush()
            self._last_status = status

    def publish(self) -> None:
        self._pub.publish(Float32(data=1.0 if self.engaged else 0.0))

    def set_engaged(self, value: bool) -> None:
        if value != self.engaged:
            self.engaged = value
            sys.stdout.write(f"\r\nclutch: {'ENGAGED' if value else 'released'}\r\n")
            sys.stdout.flush()


def main() -> int:
    if not sys.stdin.isatty():
        print("keyboard_clutch needs an interactive terminal")
        return 1
    rclpy.init()
    node = KeyboardClutch()
    reset = None
    reset_failed = False
    old = termios.tcgetattr(sys.stdin)
    sys.stdout.write("SPACE = toggle engage, 0 = release, R = reset mock arms, q = quit\r\nclutch: released\r\n")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if reset is not None and reset.poll() is not None:
                reset_failed = reset.returncode != 0
                reset = None
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == " " and reset is None and not reset_failed:
                    node.set_engaged(not node.engaged)
                elif key in ("r", "R") and reset is None:
                    node.set_engaged(False)
                    node.publish()
                    reset = subprocess.Popen([sys.executable, str(Path(__file__).with_name("reset_mock_arms.py"))])
                elif key == "0":
                    node.set_engaged(False)
                elif key in ("q", "\x03"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if reset is not None and reset.poll() is None:
            reset.terminate()
            reset.wait(timeout=5)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.engaged = False
        node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
