# Dataset format (schema v1)

One HDF5 file per recording session, many demonstrations per file, laid out like robomimic.
Reference implementation and tests: `ros2_ws/src/quest_teleop_data/quest_teleop_data/dataset_io.py`.

```
/                         attrs: schema_version, robot, arm_joint_names, hand_joint_names,
                                 action_names, rate_hz, created
/data                     attrs: total (samples over all demos)
/data/demo_N              attrs: num_samples, success, duration, rate_hz, task, notes
    timestamps            (T,)        s since the episode started
    actions               (T, 29)     [ee_target_pos(3), ee_target_quat(4), hand_joint_target(22)]
    obs/arm_joint_pos     (T, 6)      rad
    obs/arm_joint_vel     (T, 6)      rad/s
    obs/hand_joint_pos    (T, 22)     rad
    obs/ee_pos            (T, 3)      m, tool0 in base_link
    obs/ee_quat           (T, 4)      x y z w, sign-continuous within the episode
    human/keypoints       (T, 26, 3)  wrist-frame operator skeleton (OpenXR joint order)
    human/tracked         (T,)        bool
    human/engaged         (T,)        bool: operator clutch engaged
```

- **Actions are commands, not outcomes.** The arm target pose is in `base_link`; the hand targets are
  the 22 retargeted joint angles in the order of `hand_joint_names`.
- **Sampling.** All sources are sampled together at `rate_hz` (default 30 Hz) from the latest value of
  each. Samples where joint states or TF were stale are dropped and counted in `notes`.
- **`human/*` is for analysis**, e.g. studying retargeting error. Leave it out of policy observations.
- **Health check:** `ros2 run quest_teleop_data inspect_dataset FILE.hdf5` reports sample counts,
  durations, success labels, tracking coverage, NaNs, timestamp gaps and shape errors.

```python
import h5py
with h5py.File("session.hdf5") as f:
    demo = f["data/demo_0"]
    actions, joints = demo["actions"][:], demo["obs/arm_joint_pos"][:]
    ok = demo.attrs["success"]
```
