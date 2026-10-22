#!/usr/bin/env python3
"""
keyboard_clutch.py - a global teleoperation enable on the PC keyboard.

    SPACE  toggle engaged / released        0  release        q  quit

Publishes std_msgs/Float32 on /quest/clutch at 20 Hz (1.0 = engaged, 0.0 = released) and publishes 0.0
when it exits. Use it instead of the pinch gesture: pinching with the hand that also drives an arm and
fingers makes tracking worse and ties the clutch to that hand's tracking quality.

Run (workspace sourced, from a real terminal):   python3 ~/Vr-robot-teleop/tools/keyboard_clutch.py

The gesture clutch must stop publishing to the same topic: set `clutch_topic: /quest/clutch_gesture`
for every hand_inputs node (quest_hand/config/hand.yaml and your left hand_inputs). Then
`ros2 topic info /quest/clutch` should list exactly ONE publisher.
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class KeyboardClutch(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_clutch")
        self.declare_parameter("topic", "/quest/clutch")
        self.declare_parameter("rate", 20.0)
        self._pub = self.create_publisher(Float32, self.get_parameter("topic").value, 10)
        self.engaged = False
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.publish)

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
    old = termios.tcgetattr(sys.stdin)
    sys.stdout.write("SPACE = toggle engage, 0 = release, q = quit\r\nclutch: released\r\n")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == " ":
                    node.set_engaged(not node.engaged)
                elif key == "0":
                    node.set_engaged(False)
                elif key in ("q", "\x03"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.engaged = False
        node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
