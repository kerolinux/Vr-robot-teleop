"""
One-Euro filter (Casiez, Roussel, Vogel - CHI 2012) for vectors and quaternions.

An adaptive low-pass filter: when the signal moves slowly the cutoff is low
(strong smoothing, kills hand-tracking jitter); when it moves fast the cutoff
rises (little lag). Tuning recipe:
    1. beta = 0, raise min_cutoff until jitter is gone while holding still.
    2. Raise beta until fast motions stop feeling laggy.
Units here: position in metres (speed in m/s), rotation in radians (rad/s).
"""

import math

import numpy as np

from .quaternion_utils import angle_between, normalize, slerp


def _alpha(cutoff: float, dt: float) -> float:
    """Smoothing factor of a first-order low-pass with the given cutoff (Hz)."""
    tau = 1.0 / (2.0 * math.pi * cutoff)
    return 1.0 / (1.0 + tau / dt)


class OneEuroFilter:
    """One-Euro filter for an N-dimensional signal (e.g. position)."""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.0, d_cutoff: float = 1.0):
        if min_cutoff <= 0 or d_cutoff <= 0:
            raise ValueError("cutoff frequencies must be > 0")
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.reset()

    def reset(self) -> None:
        self._t = None
        self._x_raw = None
        self._x_hat = None
        self._dx_hat = None

    def __call__(self, x, t: float) -> np.ndarray:
        """Filter sample x taken at time t (seconds)."""
        x = np.asarray(x, dtype=float)
        if self._t is None:  # first sample passes straight through
            self._t, self._x_raw, self._x_hat = t, x, x.copy()
            self._dx_hat = np.zeros_like(x)
            return x.copy()

        dt = t - self._t
        if dt <= 0.0:  # duplicate/out-of-order timestamp: ignore
            return self._x_hat.copy()

        dx = (x - self._x_raw) / dt
        a_d = _alpha(self.d_cutoff, dt)
        self._dx_hat = a_d * dx + (1.0 - a_d) * self._dx_hat

        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(self._dx_hat))
        a = _alpha(cutoff, dt)
        self._x_hat = a * x + (1.0 - a) * self._x_hat

        self._t, self._x_raw = t, x
        return self._x_hat.copy()


class OneEuroQuaternionFilter:
    """One-Euro filter for unit quaternions; speed is angular speed (rad/s)."""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.0, d_cutoff: float = 1.0):
        if min_cutoff <= 0 or d_cutoff <= 0:
            raise ValueError("cutoff frequencies must be > 0")
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.reset()

    def reset(self) -> None:
        self._t = None
        self._q_raw = None
        self._q_hat = None
        self._speed_hat = 0.0

    def __call__(self, q, t: float) -> np.ndarray:
        q = normalize(q)
        if self._t is None:
            self._t, self._q_raw, self._q_hat = t, q, q.copy()
            self._speed_hat = 0.0
            return q.copy()

        dt = t - self._t
        if dt <= 0.0:
            return self._q_hat.copy()

        speed = angle_between(self._q_raw, q) / dt
        a_d = _alpha(self.d_cutoff, dt)
        self._speed_hat = a_d * speed + (1.0 - a_d) * self._speed_hat

        cutoff = self.min_cutoff + self.beta * self._speed_hat
        self._q_hat = slerp(self._q_hat, q, _alpha(cutoff, dt))

        self._t, self._q_raw = t, q
        return self._q_hat.copy()
