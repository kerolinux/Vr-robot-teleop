"""
OpenXR -> ROS coordinate frame conversion.

    OpenXR (right-handed, Y-up):  +X right,   +Y up,   -Z forward
    ROS REP-103 (right-handed):   +X forward, +Y left, +Z up

Axis mapping (a proper rotation, det = +1, so handedness is preserved):

    x_ros = -z_xr
    y_ros = -x_xr
    z_ros =  y_xr

For a quaternion, a proper basis change just applies the same linear map to
the vector part (x, y, z); the scalar part w is unchanged. This equals the
conjugation q_ros = Q_M * q_xr * Q_M^-1 and is verified in the unit tests.
"""

import numpy as np


def openxr_to_ros_position(p) -> np.ndarray:
    x, y, z = p
    return np.array([-z, -x, y], dtype=float)


def openxr_to_ros_quaternion(q) -> np.ndarray:
    """q = [x, y, z, w] in OpenXR axes -> [x, y, z, w] in ROS axes."""
    x, y, z, w = q
    return np.array([-z, -x, y, w], dtype=float)
