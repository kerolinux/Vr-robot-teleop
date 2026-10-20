import numpy as np
import pytest

from quest_teleop_data.dataset_io import (ACTION_DIM, ARM_DIM, HAND_DIM, EpisodeBuffer, list_episodes,
                                          read_episode, read_file_attrs, summarize, validate_episode,
                                          write_episode)
from quest_teleop_data.inspect_dataset import main as inspect_main

HAND_NAMES = [f"joint_{i}" for i in range(HAND_DIM)]
RATE = 30.0


def make_buffer(n=60, dt=1 / RATE, start=0.0):
    buf = EpisodeBuffer()
    for i in range(n):
        buf.append(t=start + i * dt, arm_pos=np.full(ARM_DIM, i), arm_vel=np.zeros(ARM_DIM),
                   hand_pos=np.full(HAND_DIM, 0.1), ee_pos=np.array([0.5, 0.1, 0.4]),
                   ee_quat=np.array([0.7071, -0.7071, 0, 0]), target_pos=np.array([0.5, 0.1, 0.4 + i * 1e-3]),
                   target_quat=np.array([0.7071, -0.7071, 0, 0]), hand_target=np.full(HAND_DIM, 0.2),
                   keypoints=np.zeros((26, 3)), tracked=i % 10 != 0, engaged=True)
    return buf


def test_buffer_shapes_and_action_layout():
    arrays = make_buffer(5).to_arrays()
    assert arrays["actions"].shape == (5, ACTION_DIM) == (5, 29)
    assert np.allclose(arrays["actions"][2, :3], [0.5, 0.1, 0.402])           # position first
    assert np.allclose(arrays["actions"][0, 3:7], [0.7071, -0.7071, 0, 0])    # then quaternion
    assert np.allclose(arrays["actions"][0, 7:], 0.2)                         # then 22 hand targets
    assert arrays["timestamps"].dtype == np.float64 and arrays["human/tracked"].dtype == bool


def test_round_trip_and_appending_demos(tmp_path):
    path = tmp_path / "s.hdf5"
    a = make_buffer(60).to_arrays()
    assert write_episode(path, a, success=True, rate_hz=RATE, hand_joint_names=HAND_NAMES, task="reach") == "demo_0"
    assert write_episode(path, make_buffer(30).to_arrays(), success=False, rate_hz=RATE,
                         hand_joint_names=HAND_NAMES, notes="dropped the cup") == "demo_1"
    assert list_episodes(path) == ["demo_0", "demo_1"]

    arrays, attrs = read_episode(path, "demo_0")
    for key in a:
        assert np.array_equal(arrays[key], a[key]), key
    assert attrs["success"] and attrs["num_samples"] == 60 and attrs["task"] == "reach"
    assert attrs["duration"] == pytest.approx(59 / RATE)

    fa = read_file_attrs(path)
    assert fa["schema_version"] == 1 and len(fa["hand_joint_names"]) == 22 and len(fa["action_names"]) == 29
    import h5py
    with h5py.File(path) as f:
        assert f["data"].attrs["total"] == 90


def test_demo_numbering_survives_deletions(tmp_path):
    import h5py
    path = tmp_path / "s.hdf5"
    for _ in range(3):
        write_episode(path, make_buffer(10).to_arrays(), success=True, rate_hz=RATE, hand_joint_names=HAND_NAMES)
    with h5py.File(path, "a") as f:
        del f["data"]["demo_1"]
    assert write_episode(path, make_buffer(10).to_arrays(), success=True, rate_hz=RATE,
                         hand_joint_names=HAND_NAMES) == "demo_3"   # never reuses or overwrites a name


def test_validation_detects_problems():
    good = make_buffer(60).to_arrays()
    assert validate_episode(good, RATE) == []

    bad = {k: v.copy() for k, v in good.items()}
    bad["obs/ee_pos"][5, 0] = np.nan
    assert any("NaN" in p for p in validate_episode(bad, RATE))

    gap = make_buffer(60).to_arrays()
    gap["timestamps"][30:] += 1.0
    assert any("gap" in p for p in validate_episode(gap, RATE))

    slow = make_buffer(60, dt=0.1).to_arrays()
    assert any("median step" in p for p in validate_episode(slow, RATE))

    unordered = make_buffer(60).to_arrays()
    unordered["timestamps"][10] = unordered["timestamps"][9]
    assert any("increasing" in p for p in validate_episode(unordered, RATE))

    short = {k: v[:20] for k, v in good.items()}
    short["actions"] = short["actions"][:10]
    assert any("different lengths" in p for p in validate_episode(short, RATE))


def test_summary_and_inspect_cli(tmp_path, capsys):
    path = tmp_path / "s.hdf5"
    write_episode(path, make_buffer(60).to_arrays(), success=True, rate_hz=RATE, hand_joint_names=HAND_NAMES)
    rows = summarize(path)
    assert rows[0]["samples"] == 60 and rows[0]["success"] and rows[0]["problems"] == []
    assert rows[0]["tracked"] == pytest.approx(0.9)
    assert inspect_main([str(path)]) == 0
    out = capsys.readouterr().out
    assert "demo_0" in out and "1/1 labelled successful" in out


def test_quaternion_signs_are_continuous():
    buf = EpisodeBuffer()
    q = np.array([0.7071, -0.7071, 0.0, 0.0])
    for i, sign in enumerate([1, -1, 1, -1]):        # the same rotation arriving with random signs
        buf.append(t=i / RATE, arm_pos=np.zeros(ARM_DIM), arm_vel=np.zeros(ARM_DIM), hand_pos=np.zeros(HAND_DIM),
                   ee_pos=np.zeros(3), ee_quat=sign * q, target_pos=np.zeros(3), target_quat=sign * q,
                   hand_target=np.zeros(HAND_DIM), keypoints=np.zeros((26, 3)), tracked=True, engaged=True)
    a = buf.to_arrays()
    assert np.allclose(a["obs/ee_quat"], q) and np.allclose(a["actions"][:, 3:7], q)


def test_validation_flags_an_episode_with_no_operator_motion():
    """Structurally valid, but nobody was driving: the recorder's first real run looked like this."""
    buf = EpisodeBuffer()
    for i in range(60):
        buf.append(t=i / RATE, arm_pos=np.zeros(ARM_DIM), arm_vel=np.zeros(ARM_DIM), hand_pos=np.zeros(HAND_DIM),
                   ee_pos=np.array([0.5, 0.1, 0.4]), ee_quat=np.array([0.7071, -0.7071, 0, 0]),
                   target_pos=np.array([0.5, 0.1, 0.4]), target_quat=np.array([0.7071, -0.7071, 0, 0]),
                   hand_target=np.zeros(HAND_DIM), keypoints=np.zeros((26, 3)), tracked=False, engaged=False)
    problems = validate_episode(buf.to_arrays(), RATE)
    assert any("engaged for only 0%" in p for p in problems)
    assert any("tracked for only 0%" in p for p in problems)
    assert any("never change" in p for p in problems)
