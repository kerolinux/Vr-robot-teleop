#!/usr/bin/env python3
"""
episode_recorder_node.py  (Phase 4)

Records teleoperated demonstrations to HDF5 (see dataset_io.py for the layout), optionally with a
rosbag of the raw topics alongside. The robot state, arm target, hand command and operator
skeleton are sampled together at `rate_hz` using the latest value of each source.

Control it with services (node name `episode_recorder`):
    ros2 service call /episode_recorder/start   std_srvs/srv/Trigger
    ros2 service call /episode_recorder/stop    std_srvs/srv/SetBool "{data: true}"    # true = success
    ros2 service call /episode_recorder/discard std_srvs/srv/Trigger

Notes:
  * A sample is dropped (and counted) if joint states or the end-effector TF are older than
    `max_obs_age`. The drop count is stored in the episode's `notes`.
  * If the arm target or hand command has not been published yet, the action falls back to "hold
    the current state", so you may start recording before engaging the clutch.
  * Each node run writes one file, session_<timestamp>.hdf5, and appends episodes to it.
  * Episodes are kept in memory and written on stop. If the node is shut down while recording, the
    episode is saved (unlabelled, success=false) rather than lost.
"""

import datetime
import shutil
import signal
import subprocess
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from quest_teleop_control.hand_joints import HAND_JOINT_NAMES, HAND_PREFIX
from quest_teleop_msgs.msg import HandSkeleton

from .dataset_io import ARM_JOINT_NAMES, BimanualEpisodeBuffer, EpisodeBuffer, validate_episode, write_episode

DEFAULT_BAG_TOPICS = ["/joint_states", "/tf", "/tf_static", "/teleop/target_pose",
                      "/hand_position_controller/commands", "/teleop/right_hand/keypoints",
                      "/teleop/engaged", "/quest/right_hand/pose_raw", "/quest/right_hand/skeleton_raw"]


class EpisodeRecorder(Node):
    def __init__(self) -> None:
        super().__init__("episode_recorder")
        d = self.declare_parameter
        d("output_dir", "")                 # default: ~/quest_teleop_datasets
        d("task", "teleop_demo")
        d("rate_hz", 30.0)
        d("max_obs_age", 0.25)
        d("min_samples", 15)
        d("record_rosbag", False)
        d("bag_topics", DEFAULT_BAG_TOPICS)
        d("base_frame", "base_link")
        d("ee_frame", "tool0")
        d("bimanual", False)
        d("left_base_frame", "left_base_link")
        d("left_ee_frame", "left_tool0")
        get = lambda n: self.get_parameter(n).value  # noqa: E731

        self._rate = float(get("rate_hz"))
        self._max_age = float(get("max_obs_age"))
        self._min_samples = int(get("min_samples"))
        self._task = get("task")
        self._sides = ("right", "left") if get("bimanual") else ("right",)
        self._frames = {"right": (get("base_frame"), get("ee_frame")),
                        "left": (get("left_base_frame"), get("left_ee_frame"))}
        self._buffer_type = BimanualEpisodeBuffer if get("bimanual") else EpisodeBuffer
        self._record_bag, self._bag_topics = bool(get("record_rosbag")), list(get("bag_topics"))
        if get("bimanual"):
            self._bag_topics = list(dict.fromkeys(self._bag_topics + [
                "/left/teleop/target_pose", "/left_hand_position_controller/commands",
                "/teleop/left_hand/keypoints", "/left/teleop/engaged",
                "/quest/left_hand/pose_raw", "/quest/left_hand/skeleton_raw"]))

        out = Path(get("output_dir") or "~/quest_teleop_datasets").expanduser()
        out.mkdir(parents=True, exist_ok=True)
        self._stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self._path = out / f"session_{self._stamp}.hdf5"
        self._bag_dir = out / "bags"

        # latest values of every source
        self._js = {}
        self._inputs = {side: dict(target=None, hand_cmd=None, kp=None, kp_time=None, engaged=False)
                        for side in self._sides}

        # recording state
        self._recording = False
        self._buffer = self._buffer_type()
        self._t0 = None
        self._dropped = 0
        self._bag_proc = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        for side in self._sides:
            namespace = "" if side == "right" else "/left"
            controller = "" if side == "right" else "left_"
            self.create_subscription(PoseStamped, namespace + "/teleop/target_pose",
                                     lambda msg, side=side: self._on_target(msg, side), qos_profile_sensor_data)
            self.create_subscription(Float64MultiArray, "/" + controller + "hand_position_controller/commands",
                                     lambda msg, side=side: self._on_hand_cmd(msg, side), qos_profile_sensor_data)
            self.create_subscription(HandSkeleton, f"/teleop/{side}_hand/keypoints",
                                     lambda msg, side=side: self._on_keypoints(msg, side), qos_profile_sensor_data)
            self.create_subscription(Bool, namespace + "/teleop/engaged",
                                     lambda msg, side=side: self._on_engaged(msg, side), latched)

        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(SetBool, "~/stop", self._srv_stop)
        self.create_service(Trigger, "~/discard", self._srv_discard)
        self.create_timer(1.0 / self._rate, self._sample)
        self.get_logger().info(f"Recorder ready: {self._path}  (call ~/start to begin an episode)")

    # ------------------------------------------------------------------ inputs
    def _on_joint_states(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else float("nan")
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self._js[name] = (pos, vel, self.get_clock().now())

    def _on_target(self, msg: PoseStamped, side="right") -> None:
        p, o = msg.pose.position, msg.pose.orientation
        self._inputs[side]["target"] = (np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w]))

    def _on_hand_cmd(self, msg: Float64MultiArray, side="right") -> None:
        if len(msg.data) == len(HAND_JOINT_NAMES):
            self._inputs[side]["hand_cmd"] = np.array(msg.data, dtype=float)

    def _on_keypoints(self, msg: HandSkeleton, side="right") -> None:
        if len(msg.joints) == 26:
            pos = np.array([[j.position.x, j.position.y, j.position.z] for j in msg.joints])
            self._inputs[side]["kp"] = (pos, bool(msg.tracked))
            self._inputs[side]["kp_time"] = self.get_clock().now()

    def _on_engaged(self, msg: Bool, side="right") -> None:
        self._inputs[side]["engaged"] = bool(msg.data)

    def _age(self, t) -> float:
        return float("inf") if t is None else (self.get_clock().now() - t).nanoseconds * 1e-9

    def _ee_pose(self, side):
        base_frame, ee_frame = self._frames[side]
        try:
            t = self._tf_buffer.lookup_transform(base_frame, ee_frame, Time())
        except TransformException:
            return None
        if (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9 > self._max_age:
            return None
        tr, ro = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), np.array([ro.x, ro.y, ro.z, ro.w])

    # ------------------------------------------------------------------ sampling
    def _collect_side(self, side):
        prefix = "" if side == "right" else "left_"
        arm = [self._js.get(prefix + n) for n in ARM_JOINT_NAMES]
        hand = [self._js.get(prefix + HAND_PREFIX + n) for n in HAND_JOINT_NAMES]
        if any(v is None for v in arm + hand):
            return None, f"{side}: /joint_states is missing arm or hand joints"
        # A fresh right-arm message must not hide stale left-arm joints.
        if any(self._age(v[2]) > self._max_age for v in arm + hand):
            return None, f"{side}: stale joint states"
        ee = self._ee_pose(side)
        if ee is None:
            return None, f"{side}: no fresh TF {self._frames[side]}"

        inputs = self._inputs[side]
        hand_pos = np.array([v[0] for v in hand])
        target_p, target_q = inputs["target"] if inputs["target"] is not None else ee
        kp, tracked = (np.zeros((26, 3)), False)
        if inputs["kp"] is not None and self._age(inputs["kp_time"]) <= self._max_age:
            kp, tracked = inputs["kp"]
        return dict(
            arm_pos=np.array([v[0] for v in arm]), arm_vel=np.array([v[1] for v in arm]),
            hand_pos=hand_pos, ee_pos=ee[0], ee_quat=ee[1], target_pos=target_p, target_quat=target_q,
            hand_target=inputs["hand_cmd"] if inputs["hand_cmd"] is not None else hand_pos,
            keypoints=kp, tracked=tracked, engaged=inputs["engaged"]), ""

    def _collect(self):
        """Accept both arms together, or drop the entire paired sample."""
        samples = {}
        for side in self._sides:
            sample, reason = self._collect_side(side)
            if sample is None:
                return None, reason
            samples[side] = sample
        return (samples if len(self._sides) == 2 else samples["right"]), ""

    def _sample(self) -> None:
        if not self._recording:
            return
        sample, _ = self._collect()
        if sample is None:
            self._dropped += 1
            return
        self._buffer.append(t=(self.get_clock().now() - self._t0).nanoseconds * 1e-9, **sample)
        if len(self._buffer) % int(10 * self._rate) == 0:
            self.get_logger().info(f"recording ... {len(self._buffer)} samples")

    # ------------------------------------------------------------------ rosbag (optional)
    def _start_bag(self) -> None:
        if not self._record_bag:
            return
        if shutil.which("ros2") is None:
            self.get_logger().warn("record_rosbag is on but `ros2` is not on PATH; skipping the bag")
            return
        self._bag_dir.mkdir(parents=True, exist_ok=True)
        out = self._bag_dir / f"{self._stamp}_{datetime.datetime.now().strftime('%H%M%S')}"
        self._bag_proc = subprocess.Popen(["ros2", "bag", "record", "-o", str(out), *self._bag_topics],
                                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.get_logger().info(f"rosbag -> {out} (takes ~1 s to start)")

    def _stop_bag(self) -> None:
        if self._bag_proc is None:
            return
        self._bag_proc.send_signal(signal.SIGINT)
        try:
            self._bag_proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            self._bag_proc.kill()
        self._bag_proc = None

    # ------------------------------------------------------------------ episode control
    def _reset(self) -> None:
        self._recording = False
        self._buffer = self._buffer_type()
        self._dropped = 0

    def _srv_start(self, _req, res):
        if self._recording:
            res.success, res.message = False, "already recording; call stop or discard first"
            return res
        sample, reason = self._collect()
        if sample is None:
            res.success, res.message = False, f"cannot start: {reason}"
            return res
        self._reset()
        self._t0 = self.get_clock().now()
        self._recording = True
        self._start_bag()
        res.success, res.message = True, "recording"
        self.get_logger().info("EPISODE STARTED")
        return res

    def _srv_stop(self, req, res):
        if not self._recording:
            res.success, res.message = False, "not recording"
            return res
        res.success, res.message = self._finish(bool(req.data))
        return res

    def _srv_discard(self, _req, res):
        if not self._recording:
            res.success, res.message = False, "not recording"
            return res
        self._stop_bag()
        n = len(self._buffer)
        self._reset()
        res.success, res.message = True, f"discarded {n} samples"
        self.get_logger().info(res.message)
        return res

    def _finish(self, success: bool, note: str = ""):
        self._recording = False
        self._stop_bag()
        n, dropped = len(self._buffer), self._dropped
        if n < self._min_samples:
            self._reset()
            return False, f"discarded: only {n} samples (min_samples={self._min_samples})"
        arrays = self._buffer.to_arrays()
        problems = validate_episode(arrays, self._rate)
        for p in problems:
            self.get_logger().warn(f"episode check: {p}")
        notes = "; ".join(x for x in (note, f"dropped_samples={dropped}") if x)
        try:
            name = write_episode(self._path, arrays, success=success, rate_hz=self._rate,
                                 hand_joint_names=HAND_JOINT_NAMES, task=self._task, notes=notes,
                                 base_frames=[self._frames[side][0] for side in self._sides],
                                 ee_frames=[self._frames[side][1] for side in self._sides])
        except (OSError, ValueError) as exc:
            self._reset()
            return False, f"could not write {self._path}: {exc}"
        self._reset()
        msg = (f"saved {name}: {n} samples, success={success}, {dropped} dropped"
               + (f", {len(problems)} warning(s)" if problems else ""))
        self.get_logger().info(f"EPISODE SAVED -> {self._path}  ({msg})")
        return True, msg

    def destroy_node(self) -> bool:
        if self._recording:
            ok, msg = self._finish(False, note="saved on shutdown, unlabelled")
            self.get_logger().warn(f"shutdown while recording: {msg}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EpisodeRecorder()
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
