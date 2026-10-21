import numpy as np
import pytest

from quest_hand.hand_inputs import INDEX_TIP, THUMB_TIP, pinch_strength, wrist_pose
from quest_hand.hand_model import synthetic_hand
from quest_hand.hand_protocol import POS_VALID, ROT_VALID, WRIST

ALL = np.full(26, POS_VALID | ROT_VALID, dtype=np.uint8)


def hand_with_tip_gap(gap):
    pos, quat = synthetic_hand([0.0] * 5)
    pos[THUMB_TIP] = pos[INDEX_TIP] + np.array([gap, 0.0, 0.0])
    return pos, quat


def test_pinch_is_closed_open_and_in_between():
    assert pinch_strength(hand_with_tip_gap(0.015)[0], ALL, True) == 1.0
    assert pinch_strength(hand_with_tip_gap(0.09)[0], ALL, True) == 0.0
    mid = pinch_strength(hand_with_tip_gap(0.045)[0], ALL, True)
    assert mid == pytest.approx(0.5)


def test_pinch_ignores_where_the_hand_is():
    pos, _ = hand_with_tip_gap(0.02)
    assert pinch_strength(pos + [3.0, -2.0, 1.0], ALL, True) == pytest.approx(pinch_strength(pos, ALL, True))


def test_lost_tracking_reads_as_not_pinching():
    pos, _ = hand_with_tip_gap(0.01)               # fingertips touching ...
    assert pinch_strength(pos, ALL, tracked=False) == 0.0          # ... but the hand is not tracked
    flags = ALL.copy()
    flags[THUMB_TIP] = 0
    assert pinch_strength(pos, flags, tracked=True) == 0.0         # ... or a fingertip is invalid


def test_thresholds_are_validated():
    with pytest.raises(ValueError):
        pinch_strength(hand_with_tip_gap(0.02)[0], ALL, True, open_dist=0.02, closed_dist=0.05)


def test_wrist_pose():
    pos, quat = synthetic_hand([0.0] * 5)
    pos = pos + [0.3, 1.2, -0.4]
    p, q = wrist_pose(pos, quat, ALL, True)
    assert np.allclose(p, [0.3, 1.2, -0.4]) and np.allclose(q, [0, 0, 0, 1])
    bad = ALL.copy()
    bad[WRIST] = POS_VALID                          # rotation invalid
    assert wrist_pose(pos, quat, bad, True) is None
    assert wrist_pose(pos, quat, ALL, False) is None


def test_fake_left_hand_cycle_matches_the_hysteresis_thresholds():
    """The fake sender uses 1.5 cm (closed) and 9 cm (open): must clear teleop_core's 0.8 / 0.5 thresholds."""
    assert pinch_strength(hand_with_tip_gap(0.015)[0], ALL, True) > 0.8
    assert pinch_strength(hand_with_tip_gap(0.09)[0], ALL, True) < 0.5
