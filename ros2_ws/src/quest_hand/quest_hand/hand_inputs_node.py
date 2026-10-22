#!/usr/bin/env python3
"""
hand_inputs_node.py  (Phase 5)

Turns the raw skeleton streams into the inputs the teleoperation chain expects, so the headset only
has to send one kind of packet:

    left skeleton   -> /quest/clutch                  Float32 0..1  (thumb-index pinch of `clutch_hand`)
    right skeleton  -> /quest/right_hand/pose_raw     PoseStamped   (wrist joint)   } only if
                       /quest/right_hand/pinch        Float32       (right pinch)   } publish_pose
Leave publish_pose false while testing with the Phase 1 fake sender, which publishes pose_raw itself;
set it true with the real headset (and do not run the Phase 1 sender).

Safety: tracking loss reads as 0 (clutch released). If the clutch hand's packets stop arriving, 0 is
published until they resume, so a network stall also releases the clutch.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from quest_teleop_msgs.msg import HandSkeleton

from .hand_inputs import pinch_strength, wrist_pose


class HandInputsNode(Node):
    def __init__(self) -> None:
        super().__init__("hand_inputs")
        d = self.declare_parameter
        d("left_topic", "/quest/left_hand/skeleton_raw")
        d("right_topic", "/quest/right_hand/skeleton_raw")
        d("clutch_topic", "/quest/clutch")
        d("pose_topic", "/quest/right_hand/pose_raw")
        d("pinch_topic", "/quest/right_hand/pinch")
        d("clutch_hand", "left")
        d("pinch_open_dist", 0.07)
        d("pinch_closed_dist", 0.02)
        d("publish_pose", False)
        d("pose_hand", "right")
        d("input_timeout", 0.25)
        get = lambda n: self.get_parameter(n).value  # noqa: E731

        self._clutch_hand = get("clutch_hand")
        if self._clutch_hand not in ("left", "right"):
            raise ValueError("clutch_hand must be 'left' or 'right'")
        self._open, self._closed = float(get("pinch_open_dist")), float(get("pinch_closed_dist"))
        self._publish_pose = bool(get("publish_pose"))
        self._pose_hand = get("pose_hand")
        self._timeout = float(get("input_timeout"))
        self._last_clutch_rx = None

        self._clutch_pub = self.create_publisher(Float32, get("clutch_topic"), qos_profile_sensor_data)
        self._pose_pub = self.create_publisher(PoseStamped, get("pose_topic"), qos_profile_sensor_data)
        self._pinch_pub = self.create_publisher(Float32, get("pinch_topic"), qos_profile_sensor_data)
        self.create_subscription(HandSkeleton, get("left_topic"), self._on_skeleton, qos_profile_sensor_data)
        self.create_subscription(HandSkeleton, get("right_topic"), self._on_skeleton, qos_profile_sensor_data)
        self.create_timer(0.05, self._watchdog)
        self.get_logger().info(
            f"hand_inputs ready (clutch = {self._clutch_hand}-hand pinch, publish_pose={self._publish_pose})")

    def _on_skeleton(self, msg: HandSkeleton) -> None:
        if len(msg.joints) != 26 or len(msg.flags) != 26:
            return
        pos = np.array([[j.position.x, j.position.y, j.position.z] for j in msg.joints])
        quat = np.array([[j.orientation.x, j.orientation.y, j.orientation.z, j.orientation.w]
                         for j in msg.joints])
        flags = np.array(msg.flags, dtype=np.uint8)
        pinch = pinch_strength(pos, flags, msg.tracked, self._open, self._closed)

        if msg.hand == self._clutch_hand:
            self._last_clutch_rx = self.get_clock().now()
            self._clutch_pub.publish(Float32(data=pinch))

        if self._publish_pose and msg.hand == self._pose_hand:
            self._pinch_pub.publish(Float32(data=pinch))
            wp = wrist_pose(pos, quat, flags, msg.tracked)
            if wp is not None:
                out = PoseStamped()
                out.header.stamp, out.header.frame_id = msg.header.stamp, msg.header.frame_id
                out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, wp[0])
                (out.pose.orientation.x, out.pose.orientation.y,
                 out.pose.orientation.z, out.pose.orientation.w) = map(float, wp[1])
                self._pose_pub.publish(out)

    def _watchdog(self) -> None:
        # Armed only after the clutch hand has been seen once, so a manually published clutch
        # (ros2 topic pub /quest/clutch ...) keeps working while no headset is connected.
        if self._last_clutch_rx is None:
            return
        if (self.get_clock().now() - self._last_clutch_rx).nanoseconds * 1e-9 > self._timeout:
            self._clutch_pub.publish(Float32(data=0.0))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandInputsNode()
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
