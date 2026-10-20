#!/usr/bin/env python3
"""
hand_demo.py

Opens and closes the Shadow Hand by commanding hand_position_controller directly. Use it to
check the hand's controllers and joint directions before retargeting exists (Phase 3b).

Every joint whose lower limit is 0 (the flexion joints) is driven from 0 to `curl` * its upper
limit; abduction joints stay at 0. If a finger bends the wrong way, tell me: the flexion sign
is inferred from the model's limits.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .hand_joints import HAND_JOINTS


class HandDemo(Node):
    def __init__(self) -> None:
        super().__init__("hand_demo")
        self.declare_parameter("topic", "/hand_position_controller/commands")
        self.declare_parameter("period", 4.0)   # seconds per open-close cycle
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("max_curl", 0.8)  # fraction of the joint's upper limit
        self._period = float(self.get_parameter("period").value)
        self._max_curl = float(self.get_parameter("max_curl").value)
        self._pub = self.create_publisher(Float64MultiArray, self.get_parameter("topic").value, 10)
        self._t0 = self.get_clock().now()
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self._tick)
        self.get_logger().info(f"Cycling the hand ({len(HAND_JOINTS)} joints)")

    def _tick(self) -> None:
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        curl = self._max_curl * (0.5 - 0.5 * math.cos(2.0 * math.pi * t / self._period))
        self._pub.publish(Float64MultiArray(
            data=[curl * hi if lo == 0.0 else 0.0 for _, lo, hi in HAND_JOINTS]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
