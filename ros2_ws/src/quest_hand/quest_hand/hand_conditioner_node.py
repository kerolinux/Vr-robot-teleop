#!/usr/bin/env python3
"""
hand_conditioner_node.py  (Phase 2.5)

/quest/<hand>_hand/skeleton_raw  ->  HandConditioner  ->  /teleop/<hand>_hand/keypoints
                                                      ->  /teleop/<hand>_hand/markers (RViz)

The markers draw the wrist-frame skeleton at a fixed spot (`viz_origin`) so you
can watch finger motion on its own, independent of arm motion.
"""

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from quest_teleop_msgs.msg import HandSkeleton

from .hand_processing import HandConditioner
from .hand_protocol import BONES, NUM_JOINTS, POS_VALID, ROT_VALID
from .ros_conversions import make_pose


class HandConditionerNode(Node):
    def __init__(self) -> None:
        super().__init__("hand_conditioner")
        d = self.declare_parameter
        d("hand", "right")
        d("input_topic", "/quest/right_hand/skeleton_raw")
        d("output_topic", "/teleop/right_hand/keypoints")
        d("marker_topic", "/teleop/right_hand/markers")
        d("viz_frame", "world")
        d("viz_origin", [0.4, -0.5, 0.4])
        d("filter.min_cutoff", 2.0)
        d("filter.beta", 15.0)
        d("filter.d_cutoff", 1.0)
        d("filter.reset_gap", 0.5)
        d("filter.min_fresh_fraction", 0.8)
        get = lambda n: self.get_parameter(n).value  # noqa: E731

        self._hand = get("hand")
        self._viz_frame = get("viz_frame")
        self._viz_origin = np.array(get("viz_origin"), dtype=float)
        self._cond = HandConditioner(get("filter.min_cutoff"), get("filter.beta"), get("filter.d_cutoff"),
                                     get("filter.reset_gap"), get("filter.min_fresh_fraction"))

        self.create_subscription(HandSkeleton, get("input_topic"), self._on_raw, qos_profile_sensor_data)
        self._pub = self.create_publisher(HandSkeleton, get("output_topic"), 10)
        self._marker_pub = self.create_publisher(MarkerArray, get("marker_topic"), 10)
        self.get_logger().info(f"hand_conditioner ready ({self._hand} hand)")

    def _on_raw(self, msg: HandSkeleton) -> None:
        if len(msg.joints) != NUM_JOINTS or len(msg.flags) != NUM_JOINTS:
            self.get_logger().warn("Skeleton with wrong joint count", throttle_duration_sec=2.0)
            return

        pos = np.array([[j.position.x, j.position.y, j.position.z] for j in msg.joints])
        quat = np.array([[j.orientation.x, j.orientation.y, j.orientation.z, j.orientation.w]
                         for j in msg.joints])
        flags = np.array(msg.flags, dtype=np.uint8)

        out = self._cond.process(pos, quat, flags, msg.device_time)
        if out is None:
            self.get_logger().warn("Wrist not tracked: skipping frame", throttle_duration_sec=2.0)
            return

        res = HandSkeleton()
        res.header.stamp = msg.header.stamp
        res.header.frame_id = f"{self._hand}_hand_wrist"
        res.hand = msg.hand
        res.tracked = bool(out.tracked and msg.tracked)
        res.seq = msg.seq
        res.device_time = msg.device_time
        res.joints = [make_pose(p, q) for p, q in zip(out.positions, out.quats)]
        res.flags = [(POS_VALID | ROT_VALID) if f else 0 for f in out.fresh]
        self._pub.publish(res)
        self._marker_pub.publish(self._markers(res.header.stamp, out))

    def _markers(self, stamp, out) -> MarkerArray:
        life = Duration(sec=0, nanosec=500_000_000)

        joints = Marker()
        joints.header.stamp, joints.header.frame_id = stamp, self._viz_frame
        joints.ns, joints.id = "joints", 0
        joints.type, joints.action = Marker.SPHERE_LIST, Marker.ADD
        joints.pose.orientation.w = 1.0
        joints.scale.x = joints.scale.y = joints.scale.z = 0.012
        joints.lifetime = life
        for p, fresh in zip(out.positions, out.fresh):
            q = self._viz_origin + p
            joints.points.append(Point(x=float(q[0]), y=float(q[1]), z=float(q[2])))
            # green = fresh measurement, orange = held while tracking is lost
            joints.colors.append(ColorRGBA(r=0.2, g=0.9, b=0.3, a=1.0) if fresh
                                 else ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0))

        bones = Marker()
        bones.header.stamp, bones.header.frame_id = stamp, self._viz_frame
        bones.ns, bones.id = "bones", 1
        bones.type, bones.action = Marker.LINE_LIST, Marker.ADD
        bones.pose.orientation.w = 1.0
        bones.scale.x = 0.004
        bones.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=1.0)
        bones.lifetime = life
        for a, b in BONES:
            for i in (a, b):
                q = self._viz_origin + out.positions[i]
                bones.points.append(Point(x=float(q[0]), y=float(q[1]), z=float(q[2])))

        return MarkerArray(markers=[joints, bones])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandConditionerNode()
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
