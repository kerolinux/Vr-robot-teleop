import numpy as np
import pytest

from quest_teleop_core.frame_transform import openxr_to_ros_position, openxr_to_ros_quaternion
from quest_teleop_core.quaternion_utils import normalize, rotate_vector


def test_axes_map_correctly():
    # OpenXR forward is -Z, ROS forward is +X
    assert np.allclose(openxr_to_ros_position((0, 0, -1)), (1, 0, 0))
    # OpenXR right is +X, ROS right is -Y (Y is left)
    assert np.allclose(openxr_to_ros_position((1, 0, 0)), (0, -1, 0))
    # Up is up
    assert np.allclose(openxr_to_ros_position((0, 1, 0)), (0, 0, 1))


def test_identity_quaternion_stays_identity():
    assert np.allclose(openxr_to_ros_quaternion((0, 0, 0, 1)), (0, 0, 0, 1))


def test_yaw_about_up_axis_becomes_yaw_about_z():
    s = np.sin(np.pi / 4)
    q_ros = openxr_to_ros_quaternion((0, s, 0, np.cos(np.pi / 4)))  # 90 deg about OpenXR +Y
    assert np.allclose(q_ros, (0, 0, s, np.cos(np.pi / 4)))


def test_rotation_is_consistent_with_position_mapping():
    """Rotating then mapping == mapping then rotating (the defining property)."""
    rng = np.random.default_rng(0)
    for _ in range(20):
        q_xr = normalize(rng.normal(size=4))
        v = rng.normal(size=3)
        lhs = openxr_to_ros_position(rotate_vector(q_xr, v))
        rhs = rotate_vector(openxr_to_ros_quaternion(q_xr), openxr_to_ros_position(v))
        assert np.allclose(lhs, rhs, atol=1e-9)
