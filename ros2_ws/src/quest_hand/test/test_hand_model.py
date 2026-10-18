import numpy as np

from quest_hand.hand_model import synthetic_hand


def test_shapes_and_validity():
    pos, quat = synthetic_hand([0.2, 0.4, 0.6, 0.8, 1.0])
    assert pos.shape == (26, 3) and quat.shape == (26, 4)
    assert np.isfinite(pos).all()
    assert np.allclose(np.linalg.norm(quat, axis=1), 1.0)
    assert np.allclose(pos[1], 0.0)  # wrist at the origin


def test_open_fingers_point_forward_and_curl_closes_them():
    open_p, _ = synthetic_hand([0] * 5)
    closed_p, _ = synthetic_hand([1] * 5)
    for tip, mcp in [(10, 7), (15, 12), (20, 17), (25, 22)]:
        assert open_p[tip][2] < open_p[mcp][2]  # forward is -Z
        assert np.linalg.norm(closed_p[tip] - closed_p[mcp]) < 0.8 * np.linalg.norm(open_p[tip] - open_p[mcp])
        assert closed_p[tip][1] < open_p[tip][1]  # curls toward the palm (-Y)
