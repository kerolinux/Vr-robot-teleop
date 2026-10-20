import numpy as np
import pytest

from quest_hand.hand_model import synthetic_hand
from quest_hand.hand_processing import HandConditioner
from quest_hand.hand_protocol import POS_VALID, ROT_VALID
from quest_teleop_control.hand_joints import HAND_JOINT_NAMES, HAND_JOINTS
from quest_teleop_control.hand_retargeting import RetargetConfig, limit_rate, retarget
from quest_teleop_core.quaternion_utils import multiply, normalize, rotate_vector

ALL = np.full(26, POS_VALID | ROT_VALID, dtype=np.uint8)
IDX = {n: i for i, n in enumerate(HAND_JOINT_NAMES)}


def wrist_frame(curls, wrist_p=(0.2, 1.1, -0.3), wrist_q=(0.0, 0.0, 0.0, 1.0)):
    """synthetic hand -> world pose -> real HandConditioner -> wrist-frame positions."""
    lp, lq = synthetic_hand(curls)
    wp, wq = np.array(wrist_p), normalize(wrist_q)
    pos = np.array([wp + rotate_vector(wq, p) for p in lp])
    quat = np.array([multiply(wq, q) for q in lq])
    return HandConditioner().process(pos, quat, ALL, 0.0).positions


def test_output_shape_limits_and_names():
    out = retarget(wrist_frame([0.5] * 5))
    assert out.shape == (22,) and np.isfinite(out).all()
    for v, (_, lo, hi) in zip(out, HAND_JOINTS):
        assert lo - 1e-12 <= v <= hi + 1e-12


def test_open_hand_gives_zero_flexion_and_abduction():
    out = retarget(wrist_frame([0.0] * 5))
    for name in ("index", "middle", "ring"):
        for k in (1, 2, 3, 4):
            assert out[IDX[f"{name}_finger_joint{k}"]] == pytest.approx(0.0, abs=1e-6)
    assert np.allclose(out[[IDX[f"little_finger_joint{k}"] for k in range(1, 6)]], 0.0, atol=1e-6)


def test_finger_flexion_matches_the_synthetic_hand():
    """The model bends MCP/PIP/DIP by 1.4c / 1.7c / 1.1c; retargeting must recover those angles."""
    c = 0.5
    out = retarget(wrist_frame([0.0, c, 0.0, 0.0, 0.0]))
    assert out[IDX["index_finger_joint2"]] == pytest.approx(1.4 * c, abs=0.02)
    assert out[IDX["index_finger_joint3"]] == pytest.approx(1.7 * c, abs=0.02)
    assert out[IDX["index_finger_joint4"]] == pytest.approx(1.1 * c, abs=0.02)
    assert out[IDX["index_finger_joint1"]] == pytest.approx(0.0, abs=1e-3)   # no spreading
    assert out[IDX["middle_finger_joint2"]] == pytest.approx(0.0, abs=1e-6)  # other fingers untouched


def test_each_finger_responds_to_its_own_curl_only():
    for i, name in enumerate(["index", "middle", "ring"], start=1):
        curls = [0.0] * 5
        curls[i] = 0.7
        out = retarget(wrist_frame(curls))
        assert out[IDX[f"{name}_finger_joint2"]] > 0.5
        others = [n for n in ("index", "middle", "ring") if n != name]
        assert all(out[IDX[f"{o}_finger_joint2"]] < 1e-6 for o in others)


def test_flexion_is_monotonic_in_curl():
    prev = -1.0
    for c in np.linspace(0, 1, 6):
        v = retarget(wrist_frame([0, c, c, c, c]))[IDX["index_finger_joint3"]]
        assert v >= prev - 1e-9
        prev = v


def test_thumb_responds_to_thumb_curl():
    open_t = retarget(wrist_frame([0.0] * 5))
    closed_t = retarget(wrist_frame([1.0, 0, 0, 0, 0]))
    assert closed_t[IDX["thumb_joint5"]] > open_t[IDX["thumb_joint5"]] + 0.3
    assert closed_t[IDX["thumb_joint2"]] >= open_t[IDX["thumb_joint2"]]


def test_result_does_not_depend_on_where_the_hand_is():
    rng = np.random.default_rng(7)
    a = retarget(wrist_frame([0.3, 0.6, 0.5, 0.4, 0.2]))
    b = retarget(wrist_frame([0.3, 0.6, 0.5, 0.4, 0.2], wrist_p=rng.normal(size=3),
                             wrist_q=rng.normal(size=4)))
    assert np.allclose(a, b, atol=1e-9)


def test_gains_and_sign_parameters():
    pos = wrist_frame([0.0, 0.5, 0.0, 0.0, 0.0])
    base = retarget(pos)[IDX["index_finger_joint2"]]
    assert retarget(pos, RetargetConfig(flexion_gain=0.5))[IDX["index_finger_joint2"]] == pytest.approx(0.5 * base)


def test_rate_limiter():
    out = limit_rate(np.zeros(3), np.array([1.0, -1.0, 0.01]), 0.1)
    assert np.allclose(out, [0.1, -0.1, 0.01])
