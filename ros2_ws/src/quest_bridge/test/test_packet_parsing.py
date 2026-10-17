"""Unit test for packet validation (no ROS graph needed beyond rclpy init)."""
import json

import pytest
import rclpy

from quest_bridge.udp_pose_receiver import UdpPoseReceiver


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = UdpPoseReceiver()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_valid_packet_is_published(node):
    received = []
    node._pose_pub.publish = received.append
    node._handle_packet(json.dumps({"pos": [1, 2, 3], "quat": [0, 0, 0, 2], "pinch": 0.5}).encode())
    assert len(received) == 1
    assert received[0].pose.orientation.w == pytest.approx(1.0)  # normalised


def test_malformed_packet_is_dropped(node):
    received = []
    node._pose_pub.publish = received.append
    node._handle_packet(b"not json")
    node._handle_packet(json.dumps({"pos": [1, 2], "quat": [0, 0, 0, 1]}).encode())
    assert received == []
