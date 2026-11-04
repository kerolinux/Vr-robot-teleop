import numpy as np
import pytest

from quest_teleop_core.quaternion_utils import from_yaw
from quest_teleop_core.teleop_mapper import Hysteresis, MapperConfig, TeleopMapper

HOME_P = np.array([0.4, 0.0, 0.4])
HOME_Q = np.array([0.0, 0.0, 0.0, 1.0])
IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def make(**kw):
    return TeleopMapper(MapperConfig(home_position=HOME_P, home_quaternion=HOME_Q, **kw))


def test_disengaged_returns_none():
    assert make().update([0, 0, 0], IDENT) is None


def test_no_jump_on_engage():
    m = make()
    m.engage([5.0, 5.0, 5.0], IDENT)  # hand is far from origin: irrelevant
    p, _ = m.update([5.0, 5.0, 5.0], IDENT)
    assert np.allclose(p, HOME_P)


def test_relative_motion_and_scale():
    m = make(position_scale=0.5)
    m.engage([0, 0, 0], IDENT)
    p, _ = m.update([0.1, 0.0, -0.2], IDENT)
    assert np.allclose(p, HOME_P + [0.05, 0.0, -0.1])


def test_workspace_clamp():
    m = make(workspace_min=np.array([0.2, -0.4, 0.1]), workspace_max=np.array([0.7, 0.4, 0.8]))
    m.engage([0, 0, 0], IDENT)
    p, _ = m.update([5.0, -5.0, 5.0], IDENT)
    assert np.allclose(p, [0.7, -0.4, 0.8])


def test_hold_and_reengage_without_jump():
    m = make()
    m.engage([0, 0, 0], IDENT)
    m.update([0.1, 0, 0], IDENT)
    m.disengage()
    held_p, _ = m.target
    assert np.allclose(held_p, HOME_P + [0.1, 0, 0])
    m.engage([1.0, 1.0, 1.0], IDENT)  # operator repositioned their arm
    p, _ = m.update([1.0, 1.0, 1.0], IDENT)
    assert np.allclose(p, held_p)


def test_orientation_delta_applied_to_home():
    m = make()
    m.engage([0, 0, 0], IDENT)
    _, q = m.update([0, 0, 0], from_yaw(np.pi / 2))
    assert np.allclose(q, from_yaw(np.pi / 2))


def test_base_yaw_rotates_motion():
    m = make(base_yaw_rad=np.pi / 2)
    m.engage([0, 0, 0], IDENT)
    p, _ = m.update([0.1, 0, 0], IDENT)  # hand moves +X -> robot moves +Y
    assert np.allclose(p, HOME_P + [0.0, 0.1, 0.0], atol=1e-9)


def test_hysteresis():
    h = Hysteresis(on=0.8, off=0.5)
    assert [h.update(v) for v in (0.6, 0.9, 0.6, 0.4, 0.6)] == [False, True, True, False, False]
    with pytest.raises(ValueError):
        Hysteresis(on=0.5, off=0.8)


def test_set_target_reseeds_the_hold_point():
    m = make()
    m.set_target([0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 2.0])  # quaternion gets normalised
    p, q = m.target
    assert np.allclose(p, [0.1, 0.2, 0.3]) and np.allclose(q, [0, 0, 0, 1])
    m.engage([0, 0, 0], IDENT)
    p, _ = m.update([0.0, 0.0, 0.0], IDENT)
    assert np.allclose(p, [0.1, 0.2, 0.3])  # engaging from the re-seeded target: no jump
    with pytest.raises(RuntimeError):
        m.set_target([0, 0, 0], IDENT)       # not allowed while engaged


@pytest.mark.parametrize('lateral', [-0.03, 0.03])
def test_lateral_inversion_preserves_other_axes_rotation_and_reanchor(lateral):
    normal = make()
    inverted = make(invert_lateral=True)
    anchor = np.array([1., 2., 3.])
    hand_q = from_yaw(.2)
    for mapper in (normal, inverted):
        mapper.engage(anchor, IDENT)
    p, q = normal.update(anchor + [.02, lateral, .04], hand_q)
    flipped, flipped_q = inverted.update(anchor + [.02, lateral, .04], hand_q)
    assert np.allclose(flipped - HOME_P, [.02, -lateral, .04])
    assert np.allclose(flipped[[0, 2]], p[[0, 2]])
    assert np.allclose(flipped_q, q)
    inverted.disengage()
    inverted.set_target([.45, .12, .5], hand_q)
    inverted.engage([5., 6., 7.], hand_q)
    assert np.allclose(inverted.update([5., 6., 7.], hand_q)[0], [.45, .12, .5])


def test_lateral_inversion_still_respects_workspace_limits():
    mapper = make(invert_lateral=True, workspace_min=np.array([.2, -.1, .1]),
                  workspace_max=np.array([.7, .1, .8]))
    mapper.engage([0, 0, 0], IDENT)
    assert np.allclose(mapper.update([0, -5., 0], IDENT)[0], [.4, .1, .4])


@pytest.mark.parametrize('axis', np.eye(3))
def test_mirrored_orientation_matches_lateral_reflection(axis):
    from scipy.spatial.transform import Rotation
    anchor = Rotation.from_euler('xyz', [.3, -.2, .4])
    delta = Rotation.from_rotvec(axis * .25)
    mapper = make(mirror_orientation=True)
    mapper.set_target(HOME_P, anchor.as_quat())
    mapper.engage([0, 0, 0], anchor.as_quat())
    p, q = mapper.update([0, 0, 0], (delta * anchor).as_quat())
    reflection = np.diag([1., -1., 1.])
    expected = reflection @ delta.as_matrix() @ reflection @ anchor.as_matrix()
    assert np.allclose(Rotation.from_quat(q).as_matrix(), expected)
    assert np.allclose(p, HOME_P)
    mapper.disengage()
    mapper.set_target(p, q)
    mapper.engage([1, 2, 3], (delta * anchor).as_quat())
    assert np.allclose(mapper.update([1, 2, 3], (delta * anchor).as_quat())[1], q)


@pytest.mark.parametrize('forward', [-.03, .03])
def test_forward_inversion_preserves_accepted_axes_and_orientation(forward):
    normal = make(invert_lateral=True, mirror_orientation=True)
    corrected = make(invert_forward=True, invert_lateral=True, mirror_orientation=True)
    for mapper in (normal, corrected):
        mapper.engage([1, 2, 3], IDENT)
    hand = np.array([1, 2, 3]) + [forward, .02, .04]
    p, q = normal.update(hand, from_yaw(.2))
    new_p, new_q = corrected.update(hand, from_yaw(.2))
    assert np.allclose(new_p - HOME_P, [-forward, -.02, .04])
    assert np.allclose(new_p[1:], p[1:])
    assert np.allclose(new_q, q)
    corrected.disengage()
    corrected.set_target(new_p, new_q)
    corrected.engage([5, 6, 7], IDENT)
    assert np.allclose(corrected.update([5, 6, 7], IDENT)[0], new_p)


def test_forward_inversion_respects_workspace_limits():
    mapper = make(invert_forward=True, workspace_min=np.array([.2, -.1, .1]),
                  workspace_max=np.array([.7, .1, .8]))
    mapper.engage([0, 0, 0], IDENT)
    assert np.allclose(mapper.update([5, 0, 0], IDENT)[0], [.2, 0, .4])


@pytest.mark.parametrize('axis', np.eye(3))
@pytest.mark.parametrize('angle', [-.25, .25])
def test_tilt_correction_preserves_twist_yaw_and_position(axis, angle):
    from scipy.spatial.transform import Rotation
    anchor = Rotation.from_euler('xyz', [.3, -.2, .4])
    delta = Rotation.from_rotvec(axis * angle)
    mapper = make(invert_tilt=True, mirror_orientation=True,
                  invert_forward=True, invert_lateral=True)
    mapper.set_target(HOME_P, anchor.as_quat())
    mapper.engage([1, 2, 3], anchor.as_quat())
    p, q = mapper.update([1.02, 2.03, 3.04], (delta * anchor).as_quat())
    expected = delta.inv() * anchor
    assert np.allclose(Rotation.from_quat(q).as_matrix(), expected.as_matrix())
    assert np.allclose(p, HOME_P + [-.02, -.03, .04])
    mapper.disengage()
    mapper.set_target(p, q)
    mapper.engage([5, 6, 7], (delta * anchor).as_quat())
    new_p, new_q = mapper.update([5, 6, 7], (delta * anchor).as_quat())
    assert np.allclose(new_p, p)
    assert np.allclose(new_q, q)
