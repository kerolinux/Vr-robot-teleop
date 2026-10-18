#!/usr/bin/env python3
"""
udp_hand_receiver.py  (Phase 2.5)

Receives binary hand-skeleton datagrams from the Quest (format in
hand_protocol.py / docs/hand_protocol.md) and publishes them, unmodified apart
from sanitising, as quest_teleop_msgs/HandSkeleton in the OpenXR frame.

Topics: /quest/left_hand/skeleton_raw, /quest/right_hand/skeleton_raw
"""

import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from quest_teleop_msgs.msg import HandSkeleton

from .hand_protocol import RIGHT, ProtocolError, decode_hand_packet
from .ros_conversions import make_pose


class UdpHandReceiver(Node):
    def __init__(self) -> None:
        super().__init__("udp_hand_receiver")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5006)
        self.declare_parameter("frame_id", "quest_world")
        self.declare_parameter("topic_template", "/quest/{hand}_hand/skeleton_raw")

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        self._frame_id = self.get_parameter("frame_id").value
        template = self.get_parameter("topic_template").value

        # Both publishers are created up front (not from the receive thread).
        self._pubs = {
            "left": self.create_publisher(HandSkeleton, template.format(hand="left"), qos_profile_sensor_data),
            "right": self.create_publisher(HandSkeleton, template.format(hand="right"), qos_profile_sensor_data),
        }
        self._last_seq = {}

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self._sock.bind((host, port))
        self._sock.settimeout(0.5)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f"Listening for hand packets on udp://{host}:{port}")

    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            self._handle_packet(data)

    def _handle_packet(self, data: bytes) -> None:
        try:
            pkt = decode_hand_packet(data)
        except ProtocolError as exc:
            self.get_logger().warn(f"Dropped bad hand packet: {exc}", throttle_duration_sec=2.0)
            return

        hand = "right" if pkt.hand == RIGHT else "left"
        last = self._last_seq.get(hand)
        if last is not None and ((pkt.seq - last) & 0xFFFFFFFF) > 1 and ((pkt.seq - last) & 0xFFFFFFFF) < 1000:
            self.get_logger().warn(f"{hand} hand: lost {((pkt.seq - last) & 0xFFFFFFFF) - 1} packet(s)",
                                   throttle_duration_sec=5.0)
        self._last_seq[hand] = pkt.seq

        msg = HandSkeleton()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.hand = hand
        msg.tracked = pkt.tracked
        msg.seq = pkt.seq
        msg.device_time = pkt.device_time
        msg.joints = [make_pose(p, q) for p, q in zip(pkt.positions, pkt.quats)]
        msg.flags = [int(f) for f in pkt.flags]
        self._pubs[hand].publish(msg)

    def destroy_node(self) -> bool:
        self._stop.set()
        self._sock.close()
        self._thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UdpHandReceiver()
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
