#!/usr/bin/env python3
"""
teleop_core_node.py  (Phase 2)

Pipeline:  /quest/right_hand/pose_raw  (OpenXR frame, raw)
              -> OpenXR->ROS frame conversion
              -> One-Euro filtering (position + orientation)
              -> clutch / scaling / workspace clamp   (TeleopMapper)
              -> /teleop/target_pose  (PoseStamped, robot base frame)

Safety behaviour:
  * Nothing is published until the clutch is engaged.
  * Packet watchdog: if hand data stops for `input_timeout` s, the node
    disengages and requires the clutch to be released before re-engaging.

Clutch input (`clutch_source: topic`): std_msgs/Float32 on /quest/clutch,
0..1 (e.g. left-hand pinch or grip, sent by the Quest app). For bench testing
without a headset:
    ros2 topic pub -r 10 /quest/clutch std_msgs/msg/Float32 "{data: 1.0}"
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformException, TransformListener

from .frame_transform import openxr_to_ros_position, openxr_to_ros_quaternion
from .one_euro_filter import OneEuroFilter, OneEuroQuaternionFilter
from .teleop_mapper import Hysteresis, MapperConfig, TeleopMapper


class TeleopCoreNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_core")
        self._declare_parameters()
        get = lambda name: self.get_parameter(name).value  # noqa: E731

        # --- processing blocks ---
        self._pos_filter = OneEuroFilter(
            get("filter.position.min_cutoff"), get("filter.position.beta"), get("filter.d_cutoff")
        )
        self._rot_filter = OneEuroQuaternionFilter(
            get("filter.rotation.min_cutoff"), get("filter.rotation.beta"), get("filter.d_cutoff")
        )
        self._mapper = TeleopMapper(MapperConfig(
            home_position=np.array(get("home_position"), dtype=float),
            home_quaternion=np.array(get("home_quaternion"), dtype=float),
            position_scale=float(get("position_scale")),
            track_orientation=bool(get("track_orientation")),
            base_yaw_rad=np.deg2rad(float(get("base_yaw_deg"))),
            workspace_min=np.array(get("workspace_min"), dtype=float),
            workspace_max=np.array(get("workspace_max"), dtype=float),
        ))
        self._clutch = Hysteresis(get("clutch_on_threshold"), get("clutch_off_threshold"))
        self._clutch_source = get("clutch_source")
        if self._clutch_source not in ("topic", "always"):
            raise ValueError("clutch_source must be 'topic' or 'always'")
        self._timeout = float(get("input_timeout"))
        self._require_release = bool(get("require_release_after_timeout"))
        self._base_frame = get("base_frame")
        self._use_feedback = bool(get("use_robot_feedback"))
        self._ee_frame = get("ee_frame")
        if self._use_feedback:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- state ---
        self._hand = None            # (position, quaternion) filtered, ROS frame
        self._last_rx = None         # node-clock time of last pose
        self._needs_release = False  # set by watchdog; cleared when clutch released
        self._gripper = 0.0

        # --- I/O ---
        self.create_subscription(PoseStamped, get("input_pose_topic"), self._on_pose, qos_profile_sensor_data)
        self.create_subscription(Float32, get("input_pinch_topic"), self._on_pinch, qos_profile_sensor_data)
        self.create_subscription(Float32, get("input_clutch_topic"), self._on_clutch, qos_profile_sensor_data)
        self._target_pub = self.create_publisher(PoseStamped, get("target_pose_topic"), 10)
        self._gripper_pub = self.create_publisher(Float32, get("gripper_topic"), 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._engaged_pub = self.create_publisher(Bool, get("engaged_topic"), latched)
        self._engaged_pub.publish(Bool(data=False))

        self.create_timer(0.05, self._watchdog)
        self.get_logger().info(
            f"teleop_core ready (clutch_source={self._clutch_source}, base_frame={self._base_frame})")

    def _declare_parameters(self) -> None:
        d = self.declare_parameter
        d("input_pose_topic", "/quest/right_hand/pose_raw")
        d("input_pinch_topic", "/quest/right_hand/pinch")
        d("input_clutch_topic", "/quest/clutch")
        d("target_pose_topic", "/teleop/target_pose")
        d("gripper_topic", "/teleop/gripper_cmd")
        d("engaged_topic", "/teleop/engaged")
        d("base_frame", "base_link")
        d("use_robot_feedback", False)         # anchor the target to the measured robot pose on engage
        d("ee_frame", "tool0")                  # end-effector frame used for that feedback
        d("clutch_source", "topic")            # 'topic' | 'always' (bench testing only)
        d("clutch_on_threshold", 0.8)
        d("clutch_off_threshold", 0.5)
        d("input_timeout", 0.25)
        d("require_release_after_timeout", True)  # false: re-engage by itself once fresh data returns
        d("position_scale", 1.0)
        d("track_orientation", True)
        d("base_yaw_deg", 0.0)
        d("home_position", [0.4, 0.0, 0.4])
        d("home_quaternion", [1.0, 0.0, 0.0, 0.0])   # tool pointing down (180 deg about X)
        d("workspace_min", [0.2, -0.4, 0.1])
        d("workspace_max", [0.7, 0.4, 0.8])
        d("filter.position.min_cutoff", 1.0)
        d("filter.position.beta", 10.0)
        d("filter.rotation.min_cutoff", 1.0)
        d("filter.rotation.beta", 1.0)
        d("filter.d_cutoff", 1.0)

    # ------------------------------------------------------------------ callbacks
    def _on_pose(self, msg: PoseStamped) -> None:
        o, p = msg.pose.orientation, msg.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        pos = openxr_to_ros_position((p.x, p.y, p.z))
        quat = openxr_to_ros_quaternion((o.x, o.y, o.z, o.w))
        # Filters run continuously (even while disengaged) so they are warm
        # and fully settled at the instant the operator engages the clutch.
        self._hand = (self._pos_filter(pos, t), self._rot_filter(quat, t))
        self._last_rx = self.get_clock().now()

        if self._clutch_source == "always" and not self._mapper.engaged:
            self._engage()

        if self._mapper.engaged:
            target = self._mapper.update(*self._hand)
            out = PoseStamped()
            out.header.stamp = msg.header.stamp  # keep the original timing for logging/latency
            out.header.frame_id = self._base_frame
            tp, tq = target
            out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, tp)
            (out.pose.orientation.x, out.pose.orientation.y,
             out.pose.orientation.z, out.pose.orientation.w) = map(float, tq)
            self._target_pub.publish(out)

    def _on_pinch(self, msg: Float32) -> None:
        self._gripper = float(np.clip(msg.data, 0.0, 1.0))
        if self._mapper.engaged:
            self._gripper_pub.publish(Float32(data=self._gripper))

    def _on_clutch(self, msg: Float32) -> None:
        if self._clutch_source != "topic":
            return
        pressed = self._clutch.update(float(msg.data))
        if not pressed:
            self._needs_release = False
            self._disengage("clutch released")
        elif not self._mapper.engaged and not self._needs_release:
            self._engage()

    def _watchdog(self) -> None:
        if not self._mapper.engaged or self._last_rx is None:
            return
        age = (self.get_clock().now() - self._last_rx).nanoseconds * 1e-9
        if age > self._timeout:
            # True (default): the operator must release and re-press the clutch. False: engage again
            # automatically once fresh hand data arrives. Safe because engaging re-anchors the hand to
            # the held robot target, so the robot never jumps; it only continues from where it stopped.
            self._needs_release = self._require_release
            self._disengage(f"no hand data for {age:.2f}s")

    # ------------------------------------------------------------------ helpers
    def _engage(self) -> None:
        if self._last_rx is None or (self.get_clock().now() - self._last_rx).nanoseconds * 1e-9 > self._timeout:
            # Never anchor to a stale hand pose: the hand may have moved during the gap.
            self.get_logger().warn("Not engaging: hand pose is stale", throttle_duration_sec=2.0)
            return
        if self._hand is None:
            self.get_logger().warn("Clutch pressed but no hand data yet", throttle_duration_sec=2.0)
            return
        if self._use_feedback:
            # Start from where the robot really is, so engaging never makes the target jump.
            pose = self._robot_pose()
            if pose is None:
                self.get_logger().warn(
                    f"Cannot engage: no fresh TF {self._base_frame} -> {self._ee_frame}",
                    throttle_duration_sec=2.0)
                return
            self._mapper.set_target(*pose)
        self._mapper.engage(*self._hand)
        self._engaged_pub.publish(Bool(data=True))
        self.get_logger().info("ENGAGED")

    def _robot_pose(self):
        """Measured end-effector pose in the base frame, or None if TF is missing or stale."""
        try:
            t = self._tf_buffer.lookup_transform(self._base_frame, self._ee_frame, Time())
        except TransformException:
            return None
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        if age > 1.0:
            return None
        tr, ro = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), np.array([ro.x, ro.y, ro.z, ro.w])

    def _disengage(self, reason: str) -> None:
        if not self._mapper.engaged:
            return
        self._mapper.disengage()
        self._engaged_pub.publish(Bool(data=False))
        self.get_logger().info(f"DISENGAGED ({reason})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopCoreNode()
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
