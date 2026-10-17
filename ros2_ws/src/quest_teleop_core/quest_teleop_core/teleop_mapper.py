"""
Hand-pose -> robot-target mapping with a clutch (ROS-independent, unit-tested).

Relative ("clutched") teleoperation: when the operator engages the clutch we
remember where the hand is and where the robot target is. From then on the
target moves by the *change* in hand pose (scaled), so there is never a jump
when you re-engage, and you can reposition your arm by releasing the clutch.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from .quaternion_utils import conjugate, from_yaw, multiply, normalize, rotate_vector


class Hysteresis:
    """Schmitt trigger: on above `on`, off below `off`, otherwise keeps state."""

    def __init__(self, on: float = 0.8, off: float = 0.5):
        if off >= on:
            raise ValueError("off threshold must be lower than on threshold")
        self.on, self.off, self.state = on, off, False

    def update(self, value: float) -> bool:
        if self.state and value < self.off:
            self.state = False
        elif not self.state and value > self.on:
            self.state = True
        return self.state

    def reset(self) -> None:
        self.state = False


@dataclass
class MapperConfig:
    home_position: np.ndarray                 # robot start target, base frame (m)
    home_quaternion: np.ndarray               # [x, y, z, w]
    position_scale: float = 1.0               # robot motion per metre of hand motion
    track_orientation: bool = True
    base_yaw_rad: float = 0.0                 # rotate hand motion about Z to align with robot base
    workspace_min: np.ndarray = field(default_factory=lambda: np.full(3, -np.inf))
    workspace_max: np.ndarray = field(default_factory=lambda: np.full(3, np.inf))


class TeleopMapper:
    def __init__(self, cfg: MapperConfig):
        self.cfg = cfg
        self._yaw_q = from_yaw(cfg.base_yaw_rad)
        self._target_p = np.asarray(cfg.home_position, dtype=float).copy()
        self._target_q = normalize(cfg.home_quaternion)
        self._engaged = False
        self._hand_anchor_p = self._hand_anchor_q = None
        self._robot_anchor_p = self._robot_anchor_q = None

    @property
    def engaged(self) -> bool:
        return self._engaged

    @property
    def target(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._target_p.copy(), self._target_q.copy()

    def engage(self, hand_p, hand_q) -> None:
        """Latch anchors. The robot anchor is the *current target* -> no jump."""
        self._hand_anchor_p = np.asarray(hand_p, dtype=float).copy()
        self._hand_anchor_q = normalize(hand_q)
        self._robot_anchor_p = self._target_p.copy()
        self._robot_anchor_q = self._target_q.copy()
        self._engaged = True

    def disengage(self) -> None:
        self._engaged = False  # target is held at its last value

    def update(self, hand_p, hand_q) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Return the new (position, quaternion) target, or None when disengaged."""
        if not self._engaged:
            return None

        # Position: scaled hand displacement, rotated into the robot base frame
        delta_p = rotate_vector(self._yaw_q, np.asarray(hand_p, dtype=float) - self._hand_anchor_p)
        p = self._robot_anchor_p + self.cfg.position_scale * delta_p
        p = np.clip(p, self.cfg.workspace_min, self.cfg.workspace_max)

        # Orientation: hand rotation since engage, applied to the robot's anchor orientation
        if self.cfg.track_orientation:
            dq = multiply(normalize(hand_q), conjugate(self._hand_anchor_q))
            dq = multiply(multiply(self._yaw_q, dq), conjugate(self._yaw_q))
            q = normalize(multiply(dq, self._robot_anchor_q))
        else:
            q = self._robot_anchor_q.copy()

        self._target_p, self._target_q = p, q
        return p.copy(), q.copy()
