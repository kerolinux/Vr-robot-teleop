"""
Geometric hand retargeting: human wrist-frame skeleton -> 22 Shadow Hand joint angles.
(ROS-independent, unit-tested.)

Input: joint positions (26, 3) in the conditioned wrist frame (quest_hand.hand_processing):
    +X toward the fingertips, +Y toward the thumb side, +Z out of the back of the hand,
    so flexion curls toward -Z (the palm).

Method (baseline): measure bone *directions*, not joint rotations, so it only needs positions.
For each finger the rest axis f is the metacarpal bone projected onto the palm plane, and every
later bone gets a sagittal angle  atan2(palm-ward component, component along f).  Joint flexion
is the difference of consecutive bone angles (MCP, PIP, DIP); abduction is the lateral angle of
the proximal bone relative to the metacarpal. Using atan2 (not acos) lets cumulative curl exceed
90 degrees. Results are scaled by configurable gains and clipped to the Shadow joint limits.

The thumb (5 DOF in the robot, 4 tracked joints on the human) is a heuristic and the least
reliable part: tune thumb_* in config/retarget.yaml. Joint-direction signs for the Shadow model
are inferred, so the *_sign parameters exist to flip them without touching code.
"""

import math
from dataclasses import dataclass
from typing import Dict

import numpy as np

from .hand_joints import HAND_JOINTS, HAND_JOINT_NAMES

# OpenXR index of each finger's metacarpal joint (then proximal, intermediate, distal, tip)
FINGER_BASE = {"index": 6, "middle": 11, "ring": 16, "little": 21}
THUMB_METACARPAL = 2  # then proximal, distal, tip


@dataclass
class RetargetConfig:
    flexion_gain: float = 1.0       # robot flexion = gain * human flexion
    abduction_gain: float = 1.0
    abduction_sign: float = 1.0     # flip (-1) if fingers spread the wrong way on the robot
    little_arch_gain: float = 0.25  # little-finger palm arch as a fraction of its MCP flexion
    thumb_rest_yaw: float = 0.5     # rad: thumb swing of a relaxed human hand (subtracted)
    thumb_swing_sign: float = 1.0
    thumb_gain: float = 1.0


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _unit(v, fallback):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-6 else np.asarray(fallback, dtype=float)


def _angle_between(a, b) -> float:
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-6 or nb < 1e-6:
        return 0.0
    return math.acos(float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0)))


def finger_angles(p: np.ndarray, base: int):
    """(mcp, pip, dip, abduction) in radians for the finger whose metacarpal joint is `base`."""
    meta, prox = p[base + 1] - p[base], p[base + 2] - p[base + 1]
    inter, dist = p[base + 3] - p[base + 2], p[base + 4] - p[base + 3]

    f = _unit(np.array([meta[0], meta[1], 0.0]), [1.0, 0.0, 0.0])   # rest axis in the palm plane
    lat = np.array([-f[1], f[0], 0.0])                              # toward the thumb side

    def sagittal(v):  # angle of a bone from the rest axis, positive toward the palm
        return math.atan2(-v[2], float(v @ f))

    def lateral(v):
        return math.atan2(float(v @ lat), math.hypot(float(v @ f), v[2]))

    phi_m, phi_p, phi_i, phi_d = (sagittal(v) for v in (meta, prox, inter, dist))
    return (_wrap(phi_p - phi_m), _wrap(phi_i - phi_p), _wrap(phi_d - phi_i),
            lateral(prox) - lateral(meta))


def thumb_angles(p: np.ndarray, cfg: RetargetConfig) -> Dict[str, float]:
    """Heuristic thumb mapping (see module docstring)."""
    b0 = p[THUMB_METACARPAL + 1] - p[THUMB_METACARPAL]   # metacarpal bone
    b1 = p[THUMB_METACARPAL + 2] - p[THUMB_METACARPAL + 1]  # proximal phalanx
    b2 = p[THUMB_METACARPAL + 3] - p[THUMB_METACARPAL + 2]  # distal phalanx
    swing = math.atan2(b0[1], b0[0])                          # sideways, in the palm plane
    opposition = math.atan2(-b0[2], math.hypot(b0[0], b0[1]))  # toward the palm
    g = cfg.thumb_gain
    return {
        "thumb_joint1": cfg.thumb_swing_sign * g * (swing - cfg.thumb_rest_yaw),
        "thumb_joint2": g * opposition,
        "thumb_joint3": 0.0,
        "thumb_joint4": g * _angle_between(b0, b1),
        "thumb_joint5": g * _angle_between(b1, b2),
    }


def retarget(positions, cfg: RetargetConfig = RetargetConfig()) -> np.ndarray:
    """Wrist-frame skeleton (26, 3) -> 22 joint angles in HAND_JOINTS order, within joint limits."""
    p = np.asarray(positions, dtype=float)
    q: Dict[str, float] = {}
    for name, base in FINGER_BASE.items():
        mcp, pip, dip, abd = finger_angles(p, base)
        mcp, pip, dip = (cfg.flexion_gain * a for a in (mcp, pip, dip))
        abd = cfg.abduction_sign * cfg.abduction_gain * abd
        if name == "little":
            q["little_finger_joint1"] = cfg.little_arch_gain * mcp
            q["little_finger_joint2"] = abd
            q["little_finger_joint3"], q["little_finger_joint4"], q["little_finger_joint5"] = mcp, pip, dip
        else:
            q[f"{name}_finger_joint1"] = abd
            q[f"{name}_finger_joint2"], q[f"{name}_finger_joint3"], q[f"{name}_finger_joint4"] = mcp, pip, dip
    q.update(thumb_angles(p, cfg))

    if set(q) != set(HAND_JOINT_NAMES):  # guards against a naming typo silently dropping a joint
        raise RuntimeError(f"joint mismatch: {set(q) ^ set(HAND_JOINT_NAMES)}")
    return np.array([min(max(q[n], lo), hi) for n, lo, hi in HAND_JOINTS])


def limit_rate(previous: np.ndarray, target: np.ndarray, max_step: float) -> np.ndarray:
    """Move each joint toward its target by at most max_step radians."""
    return previous + np.clip(target - previous, -max_step, max_step)
