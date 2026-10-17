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
