"""
Control inputs derived from raw (OpenXR-frame) hand skeletons. ROS-independent, unit-tested.

  * pinch_strength: thumb-tip to index-tip closeness, 0 (open) .. 1 (touching). Used as the clutch
    (left hand) so that the thresholds live in config and not in the headset app.
  * wrist_pose: the wrist joint pose, which is the arm-control input (replaces the earlier JSON path).

Distances are in metres. Defaults suit an adult hand: fingertips touch at about 1-2 cm and are
clearly apart beyond about 7 cm. Tracking loss must read as "not pinching" so that it releases the clutch.
"""

from typing import Optional, Tuple

import numpy as np

from .hand_protocol import POS_VALID, ROT_VALID, WRIST

THUMB_TIP, INDEX_TIP = 5, 10


def pinch_strength(positions, flags, tracked: bool, open_dist: float = 0.07, closed_dist: float = 0.02) -> float:
    if open_dist <= closed_dist:
        raise ValueError("open_dist must be larger than closed_dist")
    if not tracked:
        return 0.0
    if not (flags[THUMB_TIP] & POS_VALID and flags[INDEX_TIP] & POS_VALID):
        return 0.0
    d = float(np.linalg.norm(np.asarray(positions[THUMB_TIP]) - np.asarray(positions[INDEX_TIP])))
    return float(np.clip((open_dist - d) / (open_dist - closed_dist), 0.0, 1.0))


def wrist_pose(positions, quats, flags, tracked: bool) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """(position, quaternion xyzw) of the wrist, or None if it is not trustworthy."""
    if not tracked or not (flags[WRIST] & POS_VALID and flags[WRIST] & ROT_VALID):
        return None
    return np.array(positions[WRIST], dtype=float), np.array(quats[WRIST], dtype=float)
