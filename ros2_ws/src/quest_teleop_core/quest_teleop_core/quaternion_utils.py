"""
Minimal quaternion helpers (no ROS dependency, numpy only).

Convention everywhere in this package: q = [x, y, z, w] (same as ROS),
Hamilton product, unit quaternions.
"""

import math

import numpy as np


def normalize(q) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-9:
        raise ValueError("Cannot normalise a zero-length quaternion")
    return q / n


def conjugate(q) -> np.ndarray:
    """Inverse of a unit quaternion."""
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def multiply(a, b) -> np.ndarray:
    """Hamilton product a * b (apply b first, then a)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def rotate_vector(q, v) -> np.ndarray:
    """Rotate vector v by unit quaternion q."""
    qv = np.asarray(q[:3], dtype=float)
    v = np.asarray(v, dtype=float)
    t = 2.0 * np.cross(qv, v)
    return v + q[3] * t + np.cross(qv, t)


def angle_between(q0, q1) -> float:
    """Smallest rotation angle (rad) that takes q0 to q1."""
    d = min(1.0, abs(float(np.dot(q0, q1))))
    return 2.0 * math.acos(d)


def from_yaw(yaw_rad: float) -> np.ndarray:
    """Rotation of yaw_rad about +Z."""
    return np.array([0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)])


def slerp(q0, q1, t: float) -> np.ndarray:
    """Spherical interpolation along the shortest arc (t=0 -> q0, t=1 -> q1)."""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    d = float(np.dot(q0, q1))
    if d < 0.0:  # q and -q are the same rotation; take the short way round
        q1, d = -q1, -d
    if d > 0.9995:  # nearly parallel: lerp is numerically safer
        return normalize(q0 + t * (q1 - q0))
    theta0 = math.acos(d)
    s0 = math.sin(theta0)
    return normalize((math.sin((1 - t) * theta0) * q0 + math.sin(t * theta0) * q1) / s0)
