"""
Hand-skeleton conditioning (ROS-independent, unit-tested).

raw OpenXR joints -> ROS axes -> expressed in the WRIST frame -> per-joint
One-Euro filtering, with explicit handling of tracking dropouts.

Why wrist-relative: finger retargeting must not care where the arm is. The
arm is driven by the wrist pose path (quest_teleop_core); this module only
describes the *shape* of the hand.

Resulting wrist-frame axes: +X toward the fingertips, +Y toward the thumb side
(right hand), +Z out of the back of the hand.

Dropout policy (Quest optical tracking loses occluded fingers regularly):
  * invalid joint  -> hold its last filtered position, report fresh=False
  * invalid > reset_gap seconds, then valid again -> reset that joint's filter
    (otherwise it would slowly glide from a stale position to the new one)
  * frame is `tracked` only if enough joints are fresh
  * wrist invalid -> no output at all (nothing is meaningful without it)
Filtering uses the sender's device timestamps: they are evenly spaced, whereas
UDP arrival times jitter.
"""

from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from quest_teleop_core.frame_transform import openxr_to_ros_position, openxr_to_ros_quaternion
from quest_teleop_core.one_euro_filter import OneEuroFilter
from quest_teleop_core.quaternion_utils import conjugate, multiply, normalize, rotate_vector

from .hand_protocol import NUM_JOINTS, POS_VALID, ROT_VALID, WRIST


@dataclass
class ConditionedHand:
    positions: np.ndarray        # (26, 3) wrist frame, ROS axes, filtered
    quats: np.ndarray            # (26, 4) wrist frame, ROS axes (not filtered)
    fresh: np.ndarray            # (26,) bool, True if updated from a valid measurement
    tracked: bool
    wrist_position: np.ndarray   # wrist pose in the tracking frame, ROS axes (unfiltered)
    wrist_quaternion: np.ndarray


class HandConditioner:
    def __init__(self, min_cutoff: float = 2.0, beta: float = 15.0, d_cutoff: float = 1.0,
                 reset_gap: float = 0.5, min_fresh_fraction: float = 0.8, mirror_y: bool = False):
        # mirror_y: reflect the hand across its sagittal plane so a LEFT hand reads like a right hand.
        # Needed to drive the right-hand robot model from a left hand (thumb side +Y, as documented above).
        self._mirror_y = mirror_y
        self._args = (min_cutoff, beta, d_cutoff)
        self.reset_gap = reset_gap
        self.min_fresh_fraction = min_fresh_fraction
        self.reset()

    def reset(self) -> None:
        self._filters: List[OneEuroFilter] = [OneEuroFilter(*self._args) for _ in range(NUM_JOINTS)]
        self._last_valid_t: List[Optional[float]] = [None] * NUM_JOINTS
        self._last_p = np.zeros((NUM_JOINTS, 3))
        self._last_q = np.tile([0.0, 0.0, 0.0, 1.0], (NUM_JOINTS, 1))

    def process(self, positions, quats, flags, t: float) -> Optional[ConditionedHand]:
        flags = np.asarray(flags, dtype=np.uint8)
        if not (flags[WRIST] & POS_VALID and flags[WRIST] & ROT_VALID):
            return None

        p_ros = np.array([openxr_to_ros_position(p) for p in positions])
        q_ros = np.array([openxr_to_ros_quaternion(q) for q in quats])
        w_p = p_ros[WRIST]
        w_q = normalize(q_ros[WRIST])
        w_inv = conjugate(w_q)

        fresh = np.zeros(NUM_JOINTS, dtype=bool)
        for i in range(NUM_JOINTS):
            if flags[i] & POS_VALID:
                if i == WRIST:
                    self._last_p[i] = 0.0  # the wrist is the origin of its own frame
                else:
                    last_t = self._last_valid_t[i]
                    if last_t is not None and t - last_t > self.reset_gap:
                        self._filters[i].reset()
                    rel = rotate_vector(w_inv, p_ros[i] - w_p)
                    if self._mirror_y:
                        rel = rel * np.array([1.0, -1.0, 1.0])
                    self._last_p[i] = self._filters[i](rel, t)
                self._last_valid_t[i] = t
                fresh[i] = True
            if flags[i] & ROT_VALID:
                rq = normalize(multiply(w_inv, normalize(q_ros[i])))
                if self._mirror_y:  # reflection across y: rotation axis (an axial vector) becomes (-x, y, -z)
                    rq = np.array([-rq[0], rq[1], -rq[2], rq[3]])
                self._last_q[i] = -rq if rq[3] < 0 else rq  # keep w >= 0: no sign flips

        tracked = bool(fresh.mean() >= self.min_fresh_fraction)
        return ConditionedHand(self._last_p.copy(), self._last_q.copy(), fresh, tracked,
                               w_p.copy(), w_q.copy())
