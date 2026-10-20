#!/usr/bin/env python3
"""
hand_retarget_node.py  (Phase 3b)

/teleop/right_hand/keypoints (HandSkeleton, wrist frame)
    -> geometric retargeting -> per-joint rate limit
    -> /hand_position_controller/commands (Float64MultiArray, 22 joints)

Safety: the hand only follows while teleop is engaged (the same clutch that gates the arm) and
the skeleton is tracked; otherwise the last command is simply held. Set require_engaged:=false to
test the hand on its own. Do not run hand_demo at the same time: both publish to the same topic.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray
from quest_teleop_msgs.msg import HandSkeleton

from .hand_joints import HAND_JOINTS
from .hand_retargeting import RetargetConfig, limit_rate, retarget


class HandRetargetNode(Node):
    def __init__(self) -> None:
        super().__init__("hand_retarget")
        d = self.declare_parameter
        d("input_topic", "/teleop/right_hand/keypoints")
        d("output_topic", "/hand_position_controller/commands")
        d("engaged_topic", "/teleop/engaged")
        d("require_engaged", True)
        d("max_joint_speed", 8.0)  # rad/s
        for name, default in vars(RetargetConfig()).items():
            d(name, float(default))
        get = lambda n: self.get_parameter(n).value  # noqa: E731

        self._cfg = RetargetConfig(**{n: float(get(n)) for n in vars(RetargetConfig())})
        self._require_engaged = bool(get("require_engaged"))
        self._max_speed = float(get("max_joint_speed"))
        self._engaged = False
        self._cmd = np.zeros(len(HAND_JOINTS))  # the robot hand starts open
        self._last_t = None

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, get("engaged_topic"), self._on_engaged, latched)
        self.create_subscription(HandSkeleton, get("input_topic"), self._on_keypoints, 10)
        self._pub = self.create_publisher(Float64MultiArray, get("output_topic"), 10)
        self.get_logger().info(
            f"hand_retarget ready (require_engaged={self._require_engaged}, {len(HAND_JOINTS)} joints)")

    def _on_engaged(self, msg: Bool) -> None:
        self._engaged = msg.data

    def _on_keypoints(self, msg: HandSkeleton) -> None:
        if self._require_engaged and not self._engaged:
            return
        if not msg.tracked:
            self.get_logger().warn("Hand not tracked: holding fingers", throttle_duration_sec=2.0)
            return
        if len(msg.joints) != 26:
            self.get_logger().warn("Skeleton with wrong joint count", throttle_duration_sec=2.0)
            return

        pos = np.array([[j.position.x, j.position.y, j.position.z] for j in msg.joints])
        dt = 1.0 / 60.0 if self._last_t is None else float(np.clip(msg.device_time - self._last_t, 1e-3, 0.1))
        self._last_t = msg.device_time

        self._cmd = limit_rate(self._cmd, retarget(pos, self._cfg), self._max_speed * dt)
        self._pub.publish(Float64MultiArray(data=[float(x) for x in self._cmd]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandRetargetNode()
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
