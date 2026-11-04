"""ROS callback tests; publishers are captured and no controllers are launched."""
from contextlib import contextmanager
from unittest.mock import Mock

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("quest_teleop_msgs.msg")
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from quest_teleop_control.hand_joints import HAND_JOINT_NAMES
from quest_teleop_data.dataset_io import ARM_JOINT_NAMES, arm_arrays
from quest_teleop_data.episode_recorder_node import EpisodeRecorder
from quest_teleop_data.replay_node import ReplayNode
from test_bimanual import paired, save


@contextmanager
def context(tmp_path, monkeypatch, params):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setattr(Node, "create_publisher", lambda *args, **kwargs: Mock())
    args = ["--ros-args"]
    for key, value in params.items():
        args.extend(["-p", f"{key}:={value}"])
    rclpy.init(args=args, domain_id=175)
    try:
        yield
    finally:
        rclpy.shutdown()


def joints(side):
    prefix = "" if side == "right" else "left_"
    names = [prefix + n for n in ARM_JOINT_NAMES] + [prefix + "hand_" + n for n in HAND_JOINT_NAMES]
    return JointState(name=names, position=[0.1 if side == "right" else 0.2] * len(names))


def test_recorder_pairs_both_sides_and_rejects_stale_left(tmp_path, monkeypatch):
    with context(tmp_path, monkeypatch, {"output_dir": str(tmp_path), "bimanual": "true"}):
        node = EpisodeRecorder()
        monkeypatch.setattr(node, "_ee_pose", lambda side: (np.array([0.4, 0., 0.4]), np.array([0., 0., 0., 1.])))
        try:
            node._on_joint_states(joints("right"))
            assert node._collect()[0] is None
            node._on_joint_states(joints("left"))
            for side, x in (("right", 0.5), ("left", 0.6)):
                target = PoseStamped()
                target.pose.position.x = x
                target.pose.orientation.w = 1.0
                node._on_target(target, side)
                node._on_hand_cmd(Float64MultiArray(data=[x] * 22), side)
                node._on_engaged(Bool(data=True), side)
            sample, reason = node._collect()
            assert not reason
            assert sample["left"]["target_pos"][0] == 0.6
            assert sample["right"]["target_pos"][0] == 0.5
            node._buffer.append(t=0., **sample)
            assert node._buffer.to_arrays()["actions"].shape == (1, 58)
            for name, (pos, vel, stamp) in list(node._js.items()):
                if name.startswith("left_"):
                    node._js[name] = pos, vel, stamp - Duration(seconds=1)
            node._on_joint_states(joints("right"))
            assert "left: stale" in node._collect()[1]
            node._on_joint_states(joints("left"))
            monkeypatch.setattr(node, "_ee_pose", lambda side: None if side == "left" else (np.zeros(3), np.array([0., 0., 0., 1.])))
            assert "left: no fresh TF" in node._collect()[1]
        finally:
            node.destroy_node()


@pytest.mark.parametrize("bimanual", [False, True])
def test_replay_routes_actions_and_frames(tmp_path, monkeypatch, bimanual):
    path = tmp_path / "demo.hdf5"
    arrays = paired()
    if not bimanual:
        arrays = arm_arrays(arrays, 0)
    save(path, arrays)
    if not bimanual:
        # Older schema-v1 files predate the optional frame/order metadata.
        import h5py
        with h5py.File(path, "a") as f:
            for key in ("base_frames", "ee_frames", "arm_order"):
                del f.attrs[key]
    with context(tmp_path, monkeypatch, {"file": str(path), "bimanual": str(bimanual).lower()}):
        node = ReplayNode()
        try:
            node._publish(arrays["actions"][3])
            right = node._pose_pub.publish.call_args.args[0]
            assert right.header.frame_id == "base_link"
            assert right.pose.position.y == pytest.approx(0.1)
            assert len(node._hand_pub.publish.call_args.args[0].data) == 22
            if bimanual:
                left = node._left_pose_pub.publish.call_args.args[0]
                assert left.header.frame_id == "left_base_link"
                assert left.header.stamp == right.header.stamp
                assert left.pose.position.y == pytest.approx(-0.1)
                assert np.allclose(node._left_hand_pub.publish.call_args.args[0].data, -0.1)
            else:
                assert node._left_pose_pub is None
        finally:
            node.destroy_node()


def test_bimanual_replay_requires_explicit_enable(tmp_path, monkeypatch):
    path = tmp_path / "demo.hdf5"
    save(path, paired())
    with context(tmp_path, monkeypatch, {"file": str(path)}):
        with pytest.raises(ValueError, match="bimanual parameter"):
            ReplayNode()


@pytest.mark.parametrize("fault", ["nan", "length", "frame"])
def test_replay_rejects_invalid_data_before_publishing(tmp_path, monkeypatch, fault):
    import h5py
    path = tmp_path / "bad.hdf5"
    arrays = paired()
    if fault == "nan":
        arrays["actions"][0, 29] = np.nan
    elif fault == "length":
        arrays["actions"] = arrays["actions"][:-1]
    save(path, arrays)
    if fault == "frame":
        with h5py.File(path, "a") as f:
            f.attrs["base_frames"] = ["world", "left_base_link"]
    with context(tmp_path, monkeypatch, {"file": str(path), "bimanual": "true"}):
        with pytest.raises(ValueError):
            ReplayNode()
