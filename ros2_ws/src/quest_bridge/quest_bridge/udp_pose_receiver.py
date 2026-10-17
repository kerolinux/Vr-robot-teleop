#!/usr/bin/env python3
"""
udp_pose_receiver.py  (Phase 1)

Receives hand-tracking packets from the Quest over UDP and republishes them
as ROS 2 messages. No frame conversion or filtering happens here on purpose:
this node's only job is to get raw data into ROS reliably.

Expected packet (UTF-8 JSON, one datagram per sample):
    {"pos": [x, y, z], "quat": [x, y, z, w], "pinch": 0.0-1.0}
Poses are published in the raw OpenXR frame (right-handed, Y-up); the
OpenXR -> ROS conversion is added in Phase 2.

Published topics:
    ~pose_topic  (geometry_msgs/PoseStamped)  default: /quest/right_hand/pose_raw
    ~pinch_topic (std_msgs/Float32)           default: /quest/right_hand/pinch
"""

import json
import math
import socket
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32


class UdpPoseReceiver(Node):
    def __init__(self) -> None:
        super().__init__("udp_pose_receiver")

        # --- Parameters (override via launch file or --ros-args -p) ---
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5005)
        self.declare_parameter("frame_id", "quest_world")
        self.declare_parameter("pose_topic", "/quest/right_hand/pose_raw")
        self.declare_parameter("pinch_topic", "/quest/right_hand/pinch")

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        self._frame_id = self.get_parameter("frame_id").value

        # Sensor-data QoS (best effort, small queue): stale hand poses are
        # worthless, so we prefer low latency over guaranteed delivery.
        self._pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("pose_topic").value, qos_profile_sensor_data
        )
        self._pinch_pub = self.create_publisher(
            Float32, self.get_parameter("pinch_topic").value, qos_profile_sensor_data
        )

        # --- UDP socket + background receive thread ---
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.settimeout(0.5)  # lets the thread notice shutdown requests
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(f"Listening for Quest packets on udp://{host}:{port}")

    # ------------------------------------------------------------------
    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:  # socket closed during shutdown
                break
            self._handle_packet(data)

    def _handle_packet(self, data: bytes) -> None:
        """Parse, validate and publish one datagram. Bad packets are dropped."""
        try:
            msg = json.loads(data.decode("utf-8"))
            px, py, pz = (float(v) for v in msg["pos"])
            qx, qy, qz, qw = (float(v) for v in msg["quat"])
            pinch = float(msg.get("pinch", 0.0))
        except (ValueError, KeyError, TypeError, UnicodeDecodeError):
            self.get_logger().warn("Dropped malformed packet", throttle_duration_sec=2.0)
            return

        values = (px, py, pz, qx, qy, qz, qw, pinch)
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not all(math.isfinite(v) for v in values) or norm < 1e-6:
            self.get_logger().warn("Dropped non-finite/zero-quat packet", throttle_duration_sec=2.0)
            return

        pose = PoseStamped()
        # NOTE: stamped with arrival time. Quest/PC clock sync for accurate
        # latency measurement is handled in Phase 2/4.
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = px, py, pz
        pose.pose.orientation.x = qx / norm  # normalise so downstream TF never rejects it
        pose.pose.orientation.y = qy / norm
        pose.pose.orientation.z = qz / norm
        pose.pose.orientation.w = qw / norm
        self._pose_pub.publish(pose)

        self._pinch_pub.publish(Float32(data=min(max(pinch, 0.0), 1.0)))

    # ------------------------------------------------------------------
    def destroy_node(self) -> bool:
        self._stop.set()
        self._sock.close()
        self._thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UdpPoseReceiver()
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
