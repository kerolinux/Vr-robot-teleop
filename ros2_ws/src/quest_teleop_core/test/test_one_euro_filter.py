import numpy as np

from quest_teleop_core.one_euro_filter import OneEuroFilter, OneEuroQuaternionFilter
from quest_teleop_core.quaternion_utils import angle_between, from_yaw

DT = 1.0 / 90.0


def test_first_sample_passes_through():
    f = OneEuroFilter()
    assert np.allclose(f([1.0, 2.0, 3.0], 0.0), [1.0, 2.0, 3.0])


def test_jitter_is_attenuated_when_still():
    rng = np.random.default_rng(1)
    f = OneEuroFilter(min_cutoff=1.0, beta=0.0)
    raw, out = [], []
    for i in range(540):
        x = np.array([0.3, 0.0, 0.5]) + rng.normal(scale=0.002, size=3)
        raw.append(x)
        out.append(f(x, i * DT))
    assert np.std(out[90:], axis=0).mean() < 0.4 * np.std(raw[90:], axis=0).mean()


def test_beta_reduces_lag_on_fast_motion():
    def final_error(beta):
        f = OneEuroFilter(min_cutoff=1.0, beta=beta)
        for i in range(270):  # 3 s ramp at 1 m/s
            t = i * DT
            x_hat = f([t], t)
        return abs(t - x_hat[0])

    assert final_error(10.0) < 0.5 * final_error(0.0)


def test_quaternion_filter_tracks_and_stays_unit():
    f = OneEuroQuaternionFilter(min_cutoff=1.0, beta=1.0)
    for i in range(180):
        t = i * DT
        q = f(from_yaw(0.5 * t), t)  # 0.5 rad/s yaw
        assert abs(np.linalg.norm(q) - 1.0) < 1e-9
    assert angle_between(q, from_yaw(0.5 * t)) < 0.1


def test_quaternion_filter_handles_sign_flip():
    f = OneEuroQuaternionFilter()
    q = from_yaw(0.3)
    f(q, 0.0)
    out = f(-q, DT)  # same rotation, opposite sign: must not spin the long way round
    assert angle_between(out, q) < 1e-6
