#!/usr/bin/env python3
"""Headless synthetic wrist smoke test on isolated ROS domain 174, mock hardware only.

Source ROS and the workspace, then run from any directory. No headset or UDP sender is used.
Logs are retained in ros2_ws/log/teleop_smoke_*; only processes started here are stopped.
"""
import argparse
import os
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "174"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool, Trigger
from quest_teleop_data.dataset_io import read_episode, read_file_attrs, validate_episode
from tf2_ros import Buffer, TransformException, TransformListener

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--position-only", action="store_true")
    args = parser.parse_args()
    orientation = "track_orientation:=" + ("false" if args.position_only else "true")
    description = Path(get_package_share_directory("quest_teleop_description")) / "urdf/ur5e_shadow.urdf.xacro"
    xml = subprocess.check_output(["xacro", str(description), "left_hand:=true"], text=True)
    plugins = [p.text for p in ET.fromstring(xml).findall("ros2_control/hardware/plugin")]
    if len(plugins) != 4 or set(plugins) != {"mock_components/GenericSystem"}:
        raise RuntimeError(f"Refusing non-mock hardware: {plugins}")
    log_root = ROOT / "ros2_ws/log"
    log_root.mkdir(exist_ok=True)
    logs = Path(tempfile.mkdtemp(prefix="teleop_smoke_", dir=log_root))
    os.environ["ROS_LOG_DIR"] = str(logs)
    processes, handles = [], []
    node = None
    try:
        commands = [
            ["ros2", "launch", "quest_teleop_control", "control.launch.py", "launch_rviz:=false"],
            ["ros2", "launch", "quest_teleop_core", "teleop_core.launch.py",
             "debug_tf:=false", "use_robot_feedback:=true", orientation],
            ["ros2", "launch", "quest_teleop_control", "left_teleop.launch.py", orientation],
            ["ros2", "launch", "quest_teleop_data", "record.launch.py", "bimanual:=true",
             f"output_dir:={logs / 'datasets'}", "task:=synthetic_smoke"],
        ]
        for i, command in enumerate(commands):
            handle = (logs / f"launch_{i}.log").open("w")
            handles.append(handle)
            processes.append(subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        rclpy.init(args=[])
        node = rclpy.create_node("teleop_smoke")
        tf = Buffer()
        listener = TransformListener(tf, node)
        clutch = node.create_publisher(Float32, "/quest/clutch", 10)
        pubs = {side: node.create_publisher(PoseStamped, f"/quest/{side}_hand/pose_raw", 10)
                for side in ("right", "left")}
        targets, engaged, counts = {}, {}, {"right": 0, "left": 0}
        def receive(side, message):
            targets[side] = np.array([message.pose.position.x, message.pose.position.y, message.pose.position.z])
            counts[side] += 1
        for side, prefix in (("right", ""), ("left", "/left")):
            node.create_subscription(PoseStamped, prefix + "/teleop/target_pose",
                                     lambda msg, s=side: receive(s, msg), qos_profile_sensor_data)
            node.create_subscription(Bool, prefix + "/teleop/engaged",
                                     lambda msg, s=side: engaged.update({s: msg.data}), 10)
        client = node.create_client(ListControllers, "/controller_manager/list_controllers")
        deadline = time.monotonic() + 30
        while not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() > deadline:
                raise RuntimeError("Controller manager unavailable")
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done():
            raise RuntimeError("Controller query timed out")
        # Spawners may still be activating; the streaming warm-up below allows them to finish.
        def stream(seconds, position, pressed=False, send_pose=True, rotation=0., tilt=False):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                clutch.publish(Float32(data=float(pressed)))
                if send_pose:
                    msg = PoseStamped()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.header.frame_id = "quest_world"
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
                    if tilt:
                        msg.pose.orientation.x = float(np.sin(rotation / 2))
                    else:
                        msg.pose.orientation.z = float(np.sin(rotation / 2))
                    msg.pose.orientation.w = float(np.cos(rotation / 2))
                    for pub in pubs.values():
                        pub.publish(msg)
                # Drain high-rate TF plus both arm callbacks before sampling feedback.
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.)
                time.sleep(0.01)
        def measured(side):
            prefix = "" if side == "right" else "left_"
            try:
                msg = tf.lookup_transform(prefix + "base_link", prefix + "tool0", Time())
            except TransformException as exc:
                raise RuntimeError(f"No {side} robot TF") from exc
            if (node.get_clock().now().nanoseconds * 1e-9 -
                    msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9) > 0.5:
                raise RuntimeError(f"Stale {side} robot TF")
            tr = msg.transform.translation
            return np.array([tr.x, tr.y, tr.z])
        origin = np.array([0., 1., -0.3])
        expected = {"joint_state_broadcaster", "forward_position_controller", "hand_position_controller",
                    "left_forward_position_controller", "left_hand_position_controller"}
        deadline = time.monotonic() + 30
        while True:
            stream(1, origin)  # Keep the clutch released while spawners finish.
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done(), "Controller query timed out"
            active = {c.name for c in future.result().controller if c.state == "active"}
            if expected <= active:
                break
            assert time.monotonic() < deadline, f"Controllers not active: {expected - active}"
        stream(2, origin)  # Allow the pose subscriptions to discover each other.
        assert counts == {"right": 0, "left": 0}, f"Targets published while released: {counts}"
        print("PASS both arms released; five controllers active", flush=True)
        def service(kind, name, request):
            client = node.create_client(kind, name)
            try:
                if not client.wait_for_service(timeout_sec=5):
                    raise RuntimeError(f"Service unavailable: {name}")
                result = client.call_async(request)
                rclpy.spin_until_future_complete(node, result, timeout_sec=5)
                assert result.done() and result.result().success, f"{name} failed"
                return result.result()
            finally:
                node.destroy_client(client)
        service(Trigger, "/episode_recorder/start", Trigger.Request())
        # Configured operator-view correction reverses forward/lateral displacement: -X/+Y/+Z.
        for label, delta, expected_delta in [
                ("forward", [0, 0, -0.03], [-0.03, 0, 0]),
                ("right", [0.03, 0, 0], [0, 0.03, 0]),
                ("up", [0, 0.03, 0], [0, 0, 0.03])]:
            stream(0.5, origin)
            before = {s: measured(s) for s in pubs}
            stream(0.5, origin, True)
            assert all(engaged.get(s) for s in pubs), f"Failed engagement: {engaged}"
            anchor = {s: targets[s].copy() for s in pubs}
            for fraction in np.linspace(0, 1, 30):
                stream(0.025, origin + fraction * np.array(delta), True)
            stream(2, origin + delta, True)
            for side in pubs:
                target_delta = targets[side] - anchor[side]
                actual_delta = measured(side) - before[side]
                assert np.allclose(target_delta, expected_delta, atol=0.004), (side, label, target_delta)
                assert np.allclose(actual_delta, expected_delta, atol=0.012), (side, label, actual_delta)
                print(f"PASS {side} {label}: target {target_delta.round(4)}, robot {actual_delta.round(4)}", flush=True)
            stream(0.3, origin + delta)
            held_counts = counts.copy()
            stream(0.4, origin)
            assert counts == held_counts, "Targets continue after release"
        stream(0.3, origin, True)
        stream(0.5, origin, True, send_pose=False)
        assert not any(engaged.values()), f"Watchdog did not release: {engaged}"
        held_counts = counts.copy()
        stream(0.4, origin + [0.1, 0, 0], True)
        assert counts == held_counts, "Held clutch re-engaged after dropout"
        stream(0.3, origin + [0.1, 0, 0])
        before = {s: measured(s) for s in pubs}
        stream(0.4, origin + [0.1, 0, 0], True)
        assert all(engaged.values()), "Re-engagement failed"
        assert all(np.allclose(targets[s], before[s], atol=0.004) for s in pubs), "Re-engagement jump"
        stream(0.2, origin)
        print("PASS tracking loss, release latch and no-jump re-engagement", flush=True)
        # This is a diagnostic trajectory, not a successful manipulation demonstration.
        service(SetBool, "/episode_recorder/stop", SetBool.Request(data=False))
        files = list((logs / "datasets").glob("*.hdf5"))
        assert len(files) == 1, files
        arrays, attrs = read_episode(files[0], "demo_0")
        assert read_file_attrs(files[0])["schema_version"] == 2
        assert arrays["actions"].shape[1] == 58 and len(arrays["timestamps"]) > 30
        assert not attrs["success"]
        problems = validate_episode(arrays, attrs["rate_hz"])
        # No optical skeleton is supplied: the file must flag this instead of pretending it is training data.
        assert all("hand skeleton was tracked for only 0%" in p for p in problems), problems
        assert len(problems) == 2, problems
        print(f"PASS paired recording: {len(arrays['timestamps'])} samples, 58 action columns; "
              "missing skeletons correctly flagged for both arms", flush=True)
        reset = subprocess.Popen(["python3", str(ROOT / "tools/reset_mock_arms.py")])
        try:
            deadline = time.monotonic() + 40
            while reset.poll() is None:
                assert time.monotonic() < deadline, "Reset timed out"
                stream(.1, origin)  # Keep TF callbacks and the released clutch alive.
            assert reset.returncode == 0, "Reset failed"
        finally:
            if reset.poll() is None:
                reset.terminate()
                reset.wait(timeout=5)
        stream(.5, origin)
        assert not any(engaged.values()), "Reset re-engaged a clutch"
        before = {s: measured(s) for s in pubs}
        stream(.5, origin, True)
        assert all(engaged.values()), "Cannot engage after reset"
        assert all(np.allclose(targets[s], before[s], atol=.004) for s in pubs), "Reset anchor jump"
        stream(.3, origin)
        print("PASS mock reset, released clutches and no-jump re-engagement", flush=True)
        if not args.position_only:
            from scipy.spatial.transform import Rotation
            def wrist_rotation(side):
                prefix = "" if side == "right" else "left_"
                q = tf.lookup_transform(prefix + "base_link", prefix + "tool0", Time()).transform.rotation
                return Rotation.from_quat([q.x, q.y, q.z, q.w])
            stream(.5, origin, True)
            rotations = {s: wrist_rotation(s) for s in pubs}
            angle = np.deg2rad(5.)
            for value in np.linspace(0., angle, 30):
                stream(.03, origin, True, rotation=value)
            stream(2., origin, True, rotation=angle)
            # OpenXR +Z maps to base -X; lateral orientation reflection reverses it.
            expected = Rotation.from_rotvec([angle, 0., 0.])
            for side in pubs:
                actual = wrist_rotation(side) * rotations[side].inv()
                assert (actual * expected.inv()).magnitude() < np.deg2rad(2.), (side, actual.as_rotvec())
            stream(.3, origin)
            print("PASS both wrists follow a mirrored 5-degree forward-axis twist", flush=True)
            stream(.5, origin, True)
            rotations = {s: wrist_rotation(s) for s in pubs}
            for value in np.linspace(0., angle, 30):
                stream(.03, origin, True, rotation=value, tilt=True)
            stream(2., origin, True, rotation=angle, tilt=True)
            # OpenXR +X maps to base -Y; invert_tilt changes this to +Y.
            expected = Rotation.from_rotvec([0., angle, 0.])
            for side in pubs:
                actual = wrist_rotation(side) * rotations[side].inv()
                assert (actual * expected.inv()).magnitude() < np.deg2rad(2.), (side, actual.as_rotvec())
            stream(.3, origin)
            print("PASS both wrists follow corrected 5-degree tilt", flush=True)
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        for proc in processes:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        for handle in handles:
            handle.close()
        print(f"Logs: {logs}", flush=True)


if __name__ == "__main__":
    main()
