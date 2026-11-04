# Dataset formats (single-arm v1 and bimanual v2)

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

## Bimanual recording (schema v2)

Enable with `ros2 launch quest_teleop_data record.launch.py bimanual:=true`.
The default remains schema v1. Both arm streams are sampled by one recorder timer; a sample is
dropped if any required joint or either end-effector TF is missing/stale. Freshness is checked per
joint, so right-only updates cannot make old left-arm state appear fresh. This is latest-value
sampling, not hardware clock synchronisation or interpolation.

| Dataset | Shape | Ordering |
|---|---|---|
| `timestamps` | (T,) | One shared timestamp |
| `actions` | (T, 58) | Right 29, then left 29 |
| `obs/arm_joint_pos`, `obs/arm_joint_vel` | (T, 12) | Right 6, then left 6 |
| `obs/hand_joint_pos` | (T, 44) | Right 22, then left 22 |
| `obs/ee_pos` | (T, 6) | Right xyz, then left xyz |
| `obs/ee_quat` | (T, 8) | Right xyzw, then left xyzw |
| `human/keypoints` | (T, 2, 26, 3) | Arm axis: right, left |
| `human/tracked`, `human/engaged` | (T, 2) | Arm axis: right, left |

Each 29-value action is still target position 3 + quaternion 4 + finger targets 22.
Quaternions remain sign-continuous independently for each arm.
Targets and measured end-effector poses are in each arm's own base frame, not a shared world frame.

Root metadata includes `arm_order`, `base_frames`, `ee_frames`, and side-qualified joint/action names.
Default base frames are `base_link` and `left_base_link`; end-effector frames are `tool0` and
`left_tool0`. Single- and two-arm episodes cannot be appended to the same file.

Inspection reports coverage and quality problems for each arm, so a healthy right stream cannot
hide an untracked left hand. Success remains a manual episode label; there is no task-success
detector in the mock backend. Commands retain their last values while held; command receipt age
is not recorded. Skeleton loss is represented by `tracked=false`, not by dropping the sample.

Replay of schema v2 requires an explicit option:

```bash
ros2 run quest_teleop_data replay_episode --ros-args -p file:=<path> -p demo:=demo_0 -p bimanual:=true
```

Release the keyboard clutch first. Both action blocks are replayed at the same sample index and
message timestamp, using their respective frames/controllers. A frame mismatch is rejected.
Schema-v1 replay commands remain unchanged.

For training, use all 58 action columns for a bimanual policy, or
`arm_arrays(arrays, 0)` / `arm_arrays(arrays, 1)` to obtain a right/left single-arm view.
Split training and validation by complete episode, not individual adjacent samples.
These recordings do not yet contain object state, contact measurements or camera observations;
a manipulation policy still needs a physics task and an observation definition.
