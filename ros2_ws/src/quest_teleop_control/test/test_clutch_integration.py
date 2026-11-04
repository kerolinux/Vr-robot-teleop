"""Exercise real callbacks with captured publishers; no controller commands leave this test."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("launch_ros")
pytest.importorskip("quest_teleop_msgs.msg")
from geometry_msgs.msg import PoseStamped
from launch.actions import LogInfo
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32
from quest_teleop_core.teleop_core_node import TeleopCoreNode
from quest_teleop_control.hand_retarget_node import HandRetargetNode
from quest_teleop_msgs.msg import HandSkeleton

SRC = Path(__file__).resolve().parents[2]


def left_launch_nodes(monkeypatch):
    path = SRC / "quest_teleop_control/launch/left_teleop.launch.py"
    spec = importlib.util.spec_from_file_location("left_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(SRC / name))
    captured = []
    def capture(**kwargs):
        captured.append(kwargs)
        return LogInfo(msg="captured test node")
    monkeypatch.setattr(module, "Node", capture)
    module.generate_launch_description()
    return captured


def test_keyboard_is_the_only_configured_global_clutch_source(monkeypatch):
    hand = yaml.safe_load((SRC / "quest_hand/config/hand.yaml").read_text())
    core = yaml.safe_load((SRC / "quest_teleop_core/config/teleop.yaml").read_text())
    left_inputs, left_core = left_launch_nodes(monkeypatch)
    assert hand["hand_inputs"]["ros__parameters"]["clutch_topic"] != "/quest/clutch"
    assert left_inputs["parameters"][0]["clutch_topic"] != "/quest/clutch"
    for params in (core["teleop_core"]["ros__parameters"], left_core["parameters"][0]):
        assert params["clutch_source"] == "topic"
        assert params["require_release_after_timeout"] is True
    assert left_core["parameters"][0]["input_clutch_topic"] == "/quest/clutch"


@pytest.fixture(params=["right", "left"])
def core(request, monkeypatch, tmp_path):
    # Capture all ROS publishers, including constructor publications, before creating nodes.
    monkeypatch.setattr(Node, "create_publisher", lambda *args, **kwargs: Mock())
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    params = yaml.safe_load((SRC / "quest_teleop_core/config/teleop.yaml").read_text())["teleop_core"]["ros__parameters"]
    if request.param == "left":
        params = left_launch_nodes(monkeypatch)[1]["parameters"][0]
    params = {key: value for key, value in params.items() if not isinstance(value, dict)}
    params["use_robot_feedback"] = True
    declare = TeleopCoreNode._declare_parameters
    def configure(node):
        declare(node)
        node.set_parameters([Parameter(key, value=value) for key, value in params.items()])
    monkeypatch.setattr(TeleopCoreNode, "_declare_parameters", configure)
    rclpy.init(args=[], domain_id=173)
    node = TeleopCoreNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def pose(node, z=-0.3):
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    node._on_pose(msg)


def feedback(node, monkeypatch):
    measured = (np.array([0.492, 0.133, 0.488]), np.array([1., 0., 0., 0.]))
    monkeypatch.setattr(node, "_robot_pose", lambda: measured)
    return measured


def test_release_gates_pose_and_finger_commands(core, monkeypatch):
    feedback(core, monkeypatch)
    hand = HandRetargetNode()
    skeleton = HandSkeleton(tracked=True, device_time=1.0)
    skeleton.joints = [PoseStamped().pose for _ in range(26)]
    core._engaged_pub.publish.side_effect = hand._on_engaged
    try:
        pose(core)
        core._target_pub.publish.assert_not_called()
        hand._on_keypoints(skeleton)
        hand._pub.publish.assert_not_called()
        core._on_clutch(Float32(data=1.0))
        pose(core, -0.4)
        hand._on_keypoints(skeleton)
        assert core._target_pub.publish.call_count == 1
        assert hand._pub.publish.call_count == 1
        core._on_clutch(Float32(data=0.0))
        pose(core, -0.5)
        hand._on_keypoints(skeleton)
        assert not core._mapper.engaged
        assert core._target_pub.publish.call_count == 1
        assert hand._pub.publish.call_count == 1
    finally:
        hand.destroy_node()


def test_timeout_requires_release_and_reanchors(core, monkeypatch):
    measured = feedback(core, monkeypatch)
    pose(core)
    core._on_clutch(Float32(data=1.0))
    core._last_rx = core.get_clock().now() - Duration(seconds=1)
    core._watchdog()
    assert not core._mapper.engaged
    pose(core, -0.6)
    core._on_clutch(Float32(data=1.0))
    assert not core._mapper.engaged
    core._target_pub.publish.assert_not_called()
    core._on_clutch(Float32(data=0.0))
    core._on_clutch(Float32(data=1.0))
    assert core._mapper.engaged
    # At the engagement anchor, both position and orientation equal measured feedback.
    target = core._mapper.update(*core._hand)
    assert np.allclose(target[0], measured[0])
    assert np.allclose(target[1], measured[1])


def test_no_engagement_without_fresh_pose(core, monkeypatch):
    feedback(core, monkeypatch)
    core._on_clutch(Float32(data=1.0))
    assert not core._mapper.engaged
    pose(core)
    core._last_rx = core.get_clock().now() - Duration(seconds=1)
    core._on_clutch(Float32(data=1.0))
    assert not core._mapper.engaged
    core._target_pub.publish.assert_not_called()


def test_no_engagement_without_robot_tf(core):
    pose(core)
    core._on_clutch(Float32(data=1.0))
    assert not core._mapper.engaged
    core._target_pub.publish.assert_not_called()


def test_left_launch_exposes_typed_mapping_overrides(monkeypatch):
    from launch import LaunchContext
    context = LaunchContext()
    context.launch_configurations.update(track_orientation="false", base_yaw_deg="90.0")
    overrides = left_launch_nodes(monkeypatch)[1]["parameters"][1]
    assert overrides["track_orientation"].evaluate(context) is False
    assert overrides["base_yaw_deg"].evaluate(context) == 90.0


def test_clutch_reports_servo_stop_without_changing_enable(capsys):
    from types import SimpleNamespace
    path = SRC.parents[1] / "tools/keyboard_clutch.py"
    spec = importlib.util.spec_from_file_location("keyboard_clutch_check", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = SimpleNamespace(_arm_engaged={"right": True, "left": False},
        _servo={"right": "Very close to a singularity, emergency stop"},
        _pose_at={"right": 1_000_000_000}, _last_status=None, engaged=False,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_100_000_000)))
    module.KeyboardClutch.report_status(node)
    out = capsys.readouterr().out
    assert "right: engaged, wrist fresh" in out
    assert "singularity" in out
    assert "left: released, wrist missing/stale" in out
    assert node.engaged is False
    module.KeyboardClutch.report_status(node)
    assert capsys.readouterr().out == ""
