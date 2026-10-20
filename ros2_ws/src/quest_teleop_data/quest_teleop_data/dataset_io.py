"""
HDF5 demonstration storage (ROS-independent, unit-tested).

One .hdf5 file holds many demonstrations, laid out like robomimic so common imitation-learning
code can read it with little glue:

    /                         attrs: schema_version, robot, arm_joint_names, hand_joint_names,
                                     action_names, rate_hz, created
    /data                     attrs: total (number of samples over all demos)
    /data/demo_N              attrs: num_samples, success, duration, rate_hz, task, notes
        timestamps            (T,)        seconds since the episode started
        actions               (T, 29)     [ee_target_pos(3), ee_target_quat(4), hand_joint_target(22)]
        obs/arm_joint_pos     (T, 6)      rad
        obs/arm_joint_vel     (T, 6)      rad/s
        obs/hand_joint_pos    (T, 22)     rad
        obs/ee_pos            (T, 3)      m, tool0 in base_link
        obs/ee_quat           (T, 4)      x, y, z, w (sign-continuous within the episode)
        human/keypoints       (T, 26, 3)  wrist-frame hand skeleton from the operator
        human/tracked         (T,)        bool: skeleton trustworthy at this step
        human/engaged         (T,)        bool: operator clutch engaged at this step

Actions are the *commands* the operator produced (arm target pose in base_link and the 22 hand
joint targets), not the resulting robot state. human/* is kept for analysis and retargeting
research; leave it out of the observations when training a robot policy.
"""

import datetime
from typing import Dict, List, Optional

import h5py
import numpy as np

SCHEMA_VERSION = 1
ARM_DIM, HAND_DIM, NUM_KEYPOINTS = 6, 22, 26
ACTION_DIM = 3 + 4 + HAND_DIM
ARM_JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

EXPECTED_SHAPES = {
    "timestamps": (),
    "actions": (ACTION_DIM,),
    "obs/arm_joint_pos": (ARM_DIM,),
    "obs/arm_joint_vel": (ARM_DIM,),
    "obs/hand_joint_pos": (HAND_DIM,),
    "obs/ee_pos": (3,),
    "obs/ee_quat": (4,),
    "human/keypoints": (NUM_KEYPOINTS, 3),
    "human/tracked": (),
    "human/engaged": (),
}


class EpisodeBuffer:
    """Accumulates synchronised samples in memory until the episode is saved."""

    def __init__(self) -> None:
        self._rows: List[dict] = []

    @staticmethod
    def _align(q, prev):
        """q and -q are the same rotation: pick the sign closest to the previous sample."""
        q = np.asarray(q, dtype=float)
        return -q if prev is not None and float(np.dot(q, prev)) < 0.0 else q

    def __len__(self) -> int:
        return len(self._rows)

    def append(self, *, t, arm_pos, arm_vel, hand_pos, ee_pos, ee_quat,
               target_pos, target_quat, hand_target, keypoints, tracked, engaged) -> None:
        last = self._rows[-1] if self._rows else {}
        ee_quat = self._align(ee_quat, last.get("ee_quat"))
        target_quat = self._align(target_quat, last.get("action", [None] * 7)[3:7] if last else None)
        self._rows.append(dict(
            t=float(t), arm_pos=arm_pos, arm_vel=arm_vel, hand_pos=hand_pos, ee_pos=ee_pos,
            ee_quat=ee_quat, action=np.concatenate([target_pos, target_quat, hand_target]),
            keypoints=keypoints, tracked=bool(tracked), engaged=bool(engaged)))

    def to_arrays(self) -> Dict[str, np.ndarray]:
        def col(key, dtype=np.float32):
            return np.array([r[key] for r in self._rows], dtype=dtype)
        return {
            "timestamps": col("t", np.float64),
            "actions": col("action"),
            "obs/arm_joint_pos": col("arm_pos"),
            "obs/arm_joint_vel": col("arm_vel"),
            "obs/hand_joint_pos": col("hand_pos"),
            "obs/ee_pos": col("ee_pos"),
            "obs/ee_quat": col("ee_quat"),
            "human/keypoints": col("keypoints"),
            "human/tracked": col("tracked", bool),
            "human/engaged": col("engaged", bool),
        }


def validate_episode(arrays: Dict[str, np.ndarray], rate_hz: float) -> List[str]:
    """Return human-readable problems (an empty list means the episode looks healthy)."""
    problems: List[str] = []
    for key, tail in EXPECTED_SHAPES.items():
        if key not in arrays:
            problems.append(f"missing dataset {key}")
            continue
        if arrays[key].shape[1:] != tail:
            problems.append(f"{key}: shape {arrays[key].shape}, expected (T,{','.join(map(str, tail))})")
    if problems:
        return problems
    lengths = {k: len(v) for k, v in arrays.items()}
    if len(set(lengths.values())) != 1:
        return [f"datasets have different lengths: {lengths}"]
    n = lengths["timestamps"]
    if n < 2:
        return ["fewer than 2 samples"]
    for key, v in arrays.items():
        if v.dtype.kind == "f" and not np.isfinite(v).all():
            problems.append(f"{key}: contains NaN/inf")
    dt = np.diff(arrays["timestamps"])
    if (dt <= 0).any():
        problems.append("timestamps are not strictly increasing")
    else:
        if np.median(dt) > 1.2 / rate_hz or np.median(dt) < 0.8 / rate_hz:
            problems.append(f"median step {np.median(dt) * 1000:.1f} ms, expected {1000 / rate_hz:.1f} ms")
        if dt.max() > 3.0 / rate_hz:
            problems.append(f"largest gap {dt.max():.2f} s (samples were dropped)")
    # Content checks: a structurally perfect file can still hold a demonstration with no motion in it.
    engaged = float(arrays["human/engaged"].mean())
    if engaged < 0.1:
        problems.append(f"operator was engaged for only {engaged:.0%} of the episode "
                        "(the actions are just 'hold the current pose')")
    tracked = float(arrays["human/tracked"].mean())
    if tracked < 0.5:
        problems.append(f"hand skeleton was tracked for only {tracked:.0%} of the episode")
    if float(np.ptp(arrays["actions"], axis=0).max()) < 1e-6:
        problems.append("actions never change: nothing was commanded")
    return problems


def write_episode(path, arrays: Dict[str, np.ndarray], *, success: bool, rate_hz: float,
                  hand_joint_names: List[str], task: str = "", notes: str = "",
                  robot: str = "ur5e_shadow") -> str:
    """Append one demonstration to `path` (created if needed). Returns its name, e.g. 'demo_3'."""
    with h5py.File(path, "a") as f:
        if "schema_version" not in f.attrs:
            f.attrs["schema_version"] = SCHEMA_VERSION
            f.attrs["robot"] = robot
            f.attrs["rate_hz"] = float(rate_hz)
            f.attrs["created"] = datetime.datetime.now().isoformat(timespec="seconds")
            for key, names in (("arm_joint_names", ARM_JOINT_NAMES), ("hand_joint_names", hand_joint_names),
                               ("action_names", ["ee_target_x", "ee_target_y", "ee_target_z",
                                                 "ee_target_qx", "ee_target_qy", "ee_target_qz", "ee_target_qw"]
                                + [f"hand_target_{n}" for n in hand_joint_names])):
                f.attrs.create(key, data=names, dtype=h5py.string_dtype())
        data = f.require_group("data")
        existing = [int(k.split("_")[1]) for k in data.keys() if k.startswith("demo_")]
        name = f"demo_{max(existing) + 1 if existing else 0}"
        grp = data.create_group(name)
        for key, value in arrays.items():
            grp.create_dataset(key, data=value, compression="gzip", compression_opts=4)
        n = len(arrays["timestamps"])
        grp.attrs.update(num_samples=n, success=bool(success), rate_hz=float(rate_hz),
                         duration=float(arrays["timestamps"][-1] - arrays["timestamps"][0]),
                         task=task, notes=notes)
        data.attrs["total"] = int(sum(g.attrs["num_samples"] for g in data.values()))
    return name


def _text(v):
    return v.decode() if isinstance(v, bytes) else str(v)


def list_episodes(path) -> List[str]:
    with h5py.File(path, "r") as f:
        return sorted(f["data"].keys(), key=lambda k: int(k.split("_")[1]))


def read_episode(path, name: str):
    """Return (arrays, attrs) for one demonstration."""
    with h5py.File(path, "r") as f:
        grp = f["data"][name]
        arrays = {}
        grp.visititems(lambda k, o: arrays.__setitem__(k, o[()]) if isinstance(o, h5py.Dataset) else None)
        attrs = {k: (_text(v) if isinstance(v, (bytes, str)) else v) for k, v in grp.attrs.items()}
        attrs["rate_hz"] = float(attrs.get("rate_hz", f.attrs.get("rate_hz", 30.0)))
    return arrays, attrs


def read_file_attrs(path) -> dict:
    with h5py.File(path, "r") as f:
        return {k: ([_text(x) for x in v] if isinstance(v, np.ndarray) else v) for k, v in f.attrs.items()}


def summarize(path) -> List[dict]:
    rows = []
    for name in list_episodes(path):
        arrays, attrs = read_episode(path, name)
        rows.append(dict(
            name=name, samples=int(attrs["num_samples"]), duration=float(attrs["duration"]),
            success=bool(attrs["success"]), task=attrs.get("task", ""),
            tracked=float(arrays["human/tracked"].mean()), engaged=float(arrays["human/engaged"].mean()),
            problems=validate_episode(arrays, attrs["rate_hz"])))
    return rows
