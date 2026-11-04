import numpy as np
import pytest

from quest_teleop_data.dataset_io import (
    BimanualEpisodeBuffer, arm_arrays, list_episodes, read_episode, read_file_attrs,
    summarize, validate_episode, write_episode,
)
from quest_teleop_data.inspect_dataset import main as inspect_main


def sample(i, side):
    offset = 0.1 if side == "right" else -0.1
    return dict(arm_pos=np.full(6, offset), arm_vel=np.zeros(6), hand_pos=np.full(22, offset),
                ee_pos=np.array([0.4, offset, 0.4]), ee_quat=np.array([0., 0., 0., (-1.) ** i]),
                target_pos=np.array([0.4 + i * 0.001, offset, 0.4]),
                target_quat=np.array([0., 0., 0., (-1.) ** (i + 1)]),
                hand_target=np.full(22, offset), keypoints=np.full((26, 3), offset),
                tracked=True, engaged=True)


def paired(n=20):
    buffer = BimanualEpisodeBuffer()
    for i in range(n):
        buffer.append(t=i / 30, right=sample(i, "right"), left=sample(i, "left"))
    return buffer.to_arrays()


def save(path, arrays):
    return write_episode(path, arrays, success=True, rate_hz=30,
                         hand_joint_names=[f"joint{i}" for i in range(22)], task="paired_test")


def test_paired_layout_round_trip_and_independent_quaternions(tmp_path):
    arrays = paired()
    assert arrays["actions"].shape == (20, 58)
    assert arrays["obs/arm_joint_pos"].shape == (20, 12)
    assert arrays["human/keypoints"].shape == (20, 2, 26, 3)
    assert arrays["human/tracked"].shape == (20, 2)
    for i, offset in enumerate((0.1, -0.1)):
        arm = arm_arrays(arrays, i)
        assert np.allclose(arm["actions"][:, 7:], offset)
        assert np.allclose(arm["obs/ee_quat"], [0, 0, 0, 1])
        assert np.allclose(arm["actions"][:, 3:7], [0, 0, 0, -1])
    assert validate_episode(arrays, 30) == []
    path = tmp_path / "paired.hdf5"
    save(path, arrays)
    loaded, _ = read_episode(path, "demo_0")
    assert all(np.array_equal(arrays[key], loaded[key]) for key in arrays)
    attrs = read_file_attrs(path)
    assert attrs["schema_version"] == 2
    assert attrs["arm_order"] == ["right", "left"]
    assert attrs["base_frames"] == ["base_link", "left_base_link"]
    assert len(attrs["action_names"]) == 58
    assert attrs["action_names"][29] == "left/ee_target_x"
    assert inspect_main([str(path)]) == 0


def test_bad_left_data_is_not_hidden_by_good_right_data(tmp_path, capsys):
    arrays = paired()
    arrays["human/tracked"][:, 1] = False
    arrays["human/engaged"][:, 1] = False
    arrays["actions"][:, 29:] = arrays["actions"][0, 29:]
    problems = validate_episode(arrays, 30)
    assert any("left: hand skeleton" in p for p in problems)
    assert any("left: operator" in p for p in problems)
    assert any("left: actions never change" in p for p in problems)
    path = tmp_path / "bad.hdf5"
    save(path, arrays)
    assert summarize(path)[0]["per_arm"]["left"]["tracked"] == 0
    assert inspect_main([str(path)]) == 1
    assert "left: tracked=0%" in capsys.readouterr().out


def test_schema_mix_is_rejected_without_appending(tmp_path):
    path = tmp_path / "paired.hdf5"
    arrays = paired()
    save(path, arrays)
    with pytest.raises(ValueError, match="cannot mix"):
        save(path, arm_arrays(arrays, 0))
    assert list_episodes(path) == ["demo_0"]


def test_paired_shape_and_length_errors():
    arrays = paired()
    arrays["human/keypoints"] = arrays["human/keypoints"][:, :1]
    assert any("human/keypoints" in p for p in validate_episode(arrays, 30))
    arrays = paired()
    arrays["obs/arm_joint_pos"] = arrays["obs/arm_joint_pos"][:-1]
    assert any("different lengths" in p for p in validate_episode(arrays, 30))
