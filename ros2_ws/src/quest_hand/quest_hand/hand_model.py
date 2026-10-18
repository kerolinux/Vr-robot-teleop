"""
Procedural right-hand skeleton for testing without a headset.

Hand-local frame (OpenXR axes, wrist at origin): fingers extend along -Z,
palm faces -Y (palm down), thumb is on the -X side. Dimensions are roughly an
adult hand (~19 cm wrist-to-middle-fingertip). Not anatomically exact: it only
needs to be plausible enough to exercise the pipeline and the retargeting.
"""

import math

import numpy as np
from quest_teleop_core.quaternion_utils import multiply, rotate_vector

FORWARD = np.array([0.0, 0.0, -1.0])

# (name, first joint index, lateral x offset, length scale)
_FINGERS = [
    ("index", 6, -0.025, 1.00),
    ("middle", 11, -0.005, 1.08),
    ("ring", 16, 0.015, 1.00),
    ("little", 21, 0.033, 0.80),
]
_SEG = (0.055, 0.045, 0.027, 0.020)       # metacarpal, proximal, intermediate, distal bone (m)
_BEND_MAX = (1.4, 1.7, 1.1)               # MCP, PIP, DIP flexion at full curl (rad)
_THUMB_SEG = (0.040, 0.035, 0.030)
_THUMB_BEND_MAX = (0.6, 0.9)
_THUMB_YAW = 0.5


def _qx(a: float) -> np.ndarray:
    return np.array([math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)])


def _qy(a: float) -> np.ndarray:
    return np.array([0.0, math.sin(a / 2), 0.0, math.cos(a / 2)])


def _finger(x: float, scale: float, curl: float):
    """5 joints: metacarpal, proximal(MCP), intermediate(PIP), distal(DIP), tip."""
    m, p, d = (b * curl for b in _BEND_MAX)
    cum = [0.0, m, m + p, m + p + d]          # cumulative flexion of bones 0..3
    pts = [np.array([x, 0.0, -0.025])]
    quats = []
    for k in range(4):
        q = _qx(-cum[k])                      # negative: curl toward the palm (-Y)
        quats.append(q)
        pts.append(pts[-1] + scale * _SEG[k] * rotate_vector(q, FORWARD))
    quats.append(quats[-1])                   # tip shares the distal orientation
    return np.array(pts), np.array(quats)


def _thumb(curl: float):
    """4 joints: metacarpal, proximal, distal, tip."""
    a, b = (v * curl for v in _THUMB_BEND_MAX)
    cum = [0.0, a, a + b]
    pts = [np.array([-0.02, -0.005, -0.015])]
    quats = []
    for k in range(3):
        q = multiply(_qy(_THUMB_YAW), _qx(-cum[k]))
        quats.append(q)
        pts.append(pts[-1] + _THUMB_SEG[k] * rotate_vector(q, FORWARD))
    quats.append(quats[-1])
    return np.array(pts), np.array(quats)


def synthetic_hand(curls):
    """curls = (thumb, index, middle, ring, little), each 0 (open) .. 1 (closed).

    Returns (positions (26,3), quaternions (26,4)) in the hand-local frame.
    """
    curls = np.clip(np.asarray(curls, dtype=float), 0.0, 1.0)
    pos = np.zeros((26, 3))
    quat = np.tile([0.0, 0.0, 0.0, 1.0], (26, 1))
    pos[0] = [0.0, 0.0, -0.05]                # palm centre; wrist (1) stays at the origin

    p, q = _thumb(curls[0])
    pos[2:6], quat[2:6] = p, q
    for (_, base, x, scale), c in zip(_FINGERS, curls[1:]):
        p, q = _finger(x, scale, c)
        pos[base:base + 5], quat[base:base + 5] = p, q
    return pos, quat
