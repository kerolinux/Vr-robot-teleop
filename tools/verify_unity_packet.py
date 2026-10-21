#!/usr/bin/env python3
"""
Cross-language protocol check: the C# encoder (what the Quest app uses) must produce bytes that the
Python reference decoder reads back exactly. Also verifies the Unity -> OpenXR rotation conversion
against explicit rotation matrices.

    python3 tools/verify_unity_packet.py        (needs the .NET SDK for the encoder half)
"""

import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "ros2_ws/src/quest_hand"))
from quest_hand.hand_protocol import PACKET_SIZE, RIGHT, decode_hand_packet  # noqa: E402


def quat_to_matrix(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def check_rotation_conversion() -> bool:
    """Unity quaternion (left-handed) -> OpenXR quaternion (right-handed): R' = S R S, S = diag(1,1,-1)."""
    S = np.diag([1.0, 1.0, -1.0])
    rng = np.random.default_rng(0)
    for _ in range(200):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        converted = np.array([-q[0], -q[1], q[2], q[3]])          # what FrameConversion.RotationToOpenXr does
        if not np.allclose(quat_to_matrix(converted), S @ quat_to_matrix(q) @ S, atol=1e-9):
            return False
    return True


def check_encoder() -> bool:
    if shutil.which("dotnet") is None:
        print("SKIP  C# encoder check (dotnet not installed)")
        return True
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "packet.bin"
        empty = Path(tmp) / "no_nuget_packages_needed"   # the project has no packages: restore needs no network
        empty.mkdir()
        run = subprocess.run(["dotnet", "run", "--project", str(REPO / "quest_app/Tests/EncoderCheck"),
                              f"-p:RestoreSources={empty}", "--", str(out)], capture_output=True, text=True,
                             env={**os.environ, "DOTNET_NOLOGO": "1", "DOTNET_SKIP_FIRST_TIME_EXPERIENCE": "1"})
        if run.returncode != 0:
            print(run.stdout, run.stderr)
            return False
        data = out.read_bytes()

    ok = len(data) == PACKET_SIZE
    pkt = decode_hand_packet(data)
    ok &= (pkt.hand, pkt.tracked, pkt.seq) == (RIGHT, True, 123456)
    ok &= math.isclose(pkt.device_time, 1234.5678, abs_tol=1e-9)
    k = np.arange(26)
    expected_pos = np.stack([0.01 * (k + 1) + 0.001 * j for j in range(3)], axis=1)
    theta = 0.05 * (k + 1)
    expected_quat = np.stack([np.zeros(26), np.sin(theta / 2), np.zeros(26), np.cos(theta / 2)], axis=1)
    expected_flags = np.where(k % 3 == 0, 0, 0xF)
    ok &= np.allclose(pkt.positions, expected_pos, atol=1e-6)
    ok &= np.allclose(pkt.quats, expected_quat, atol=1e-6)
    ok &= bool((pkt.flags == expected_flags).all())
    return bool(ok)


if __name__ == "__main__":
    results = {"Unity -> OpenXR rotation conversion": check_rotation_conversion(),
               "C# packet bytes decode to the same values": check_encoder()}
    for name, ok in results.items():
        print(("PASS  " if ok else "FAIL  ") + name)
    sys.exit(0 if all(results.values()) else 1)
