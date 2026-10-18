import numpy as np

from quest_hand.hand_model import synthetic_hand
from quest_hand.hand_processing import HandConditioner
from quest_hand.hand_protocol import ROT_VALID, POS_VALID, WRIST
from quest_teleop_core.frame_transform import openxr_to_ros_position
from quest_teleop_core.quaternion_utils import multiply, normalize, rotate_vector

ALL = np.full(26, POS_VALID | ROT_VALID, dtype=np.uint8)
CURLS = [0.3, 0.6, 0.5, 0.4, 0.2]


def world_hand(wrist_p, wrist_q, curls=CURLS):
    lp, lq = synthetic_hand(curls)
    pos = np.array([wrist_p + rotate_vector(wrist_q, p) for p in lp])
    quat = np.array([multiply(wrist_q, q) for q in lq])
    return pos, quat


def test_output_is_independent_of_where_the_hand_is():
    """Same finger shape at two very different wrist poses -> identical wrist-frame output."""
    rng = np.random.default_rng(3)
    results = []
    for _ in range(2):
        wp = rng.normal(size=3)
        wq = normalize(rng.normal(size=4))
        pos, quat = world_hand(wp, wq)
        results.append(HandConditioner().process(pos, quat, ALL, 0.0).positions)
    assert np.allclose(results[0], results[1], atol=1e-9)


def test_wrist_frame_axes_follow_the_documented_convention():
    pos, quat = world_hand(np.array([0.2, 1.1, -0.3]), np.array([0.0, 0.0, 0.0, 1.0]))
    out = HandConditioner().process(pos, quat, ALL, 0.0)
    local, _ = synthetic_hand(CURLS)
    expected = np.array([openxr_to_ros_position(p) for p in local])
    assert np.allclose(out.positions, expected, atol=1e-9)
    # fingertips are in front of the wrist (+X); thumb side is +Y
    assert out.positions[15][0] > 0.1 and out.positions[5][1] > 0


def test_invalid_joint_is_held_and_marked_stale():
    c = HandConditioner()
    pos, quat = world_hand(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
    first = c.process(pos, quat, ALL, 0.0)
    flags = ALL.copy()
    flags[8] = 0
    pos2 = pos.copy()
    pos2[8] = 0.0
    second = c.process(pos2, quat, flags, 0.016)
    assert not second.fresh[8] and second.fresh[7]
    assert np.allclose(second.positions[8], first.positions[8])


def test_tracked_false_when_too_many_joints_are_lost():
    c = HandConditioner(min_fresh_fraction=0.8)
    pos, quat = world_hand(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
    flags = ALL.copy()
    flags[6:16] = 0
    assert c.process(pos, quat, flags, 0.0).tracked is False
    assert c.process(pos, quat, ALL, 0.016).tracked is True


def test_wrist_lost_gives_no_output():
    c = HandConditioner()
    pos, quat = world_hand(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))
    flags = ALL.copy()
    flags[WRIST] = 0
    assert c.process(pos, quat, flags, 0.0) is None


def test_filter_restarts_after_a_long_dropout():
    c = HandConditioner(reset_gap=0.5)
    ident = np.array([0.0, 0.0, 0.0, 1.0])
    pos, quat = world_hand(np.zeros(3), ident)
    c.process(pos, quat, ALL, 0.0)
    lost = ALL.copy()
    lost[8] = 0
    for k in range(1, 20):                       # joint 8 lost for ~1 s
        c.process(pos, quat, lost, 0.05 * k)
    moved = pos.copy()
    moved[8] += [0.0, 0.0, -0.05]                # reappears 5 cm away
    out = c.process(moved, quat, ALL, 1.05)
    expected = openxr_to_ros_position(moved[8] - pos[WRIST])
    assert np.allclose(out.positions[8], expected, atol=1e-9)  # snapped, not glided
