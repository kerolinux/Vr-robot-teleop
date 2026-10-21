#!/usr/bin/env python3
"""
fake_hand_sender.py

Stand-in for the Quest app: streams a synthetic right hand (wrist circling,
fingers opening and closing in a wave) using the real binary protocol.

    ros2 run quest_hand fake_hand_sender [--host 127.0.0.1] [--port 5006] [--hz 60] [--dropout]

--dropout makes the index finger lose tracking for 0.5 s every 6 s, to exercise
the hold/reset logic in the conditioner (it should turn orange in RViz).
--left-clutch also streams a left hand whose thumb-index pinch is closed for 75% of every
--clutch-period seconds (default 20), which hand_inputs turns into /quest/clutch. This replaces
the manual `ros2 topic pub /quest/clutch ...` when testing the real headset path.
"""

import argparse
import math
import socket
import time

import numpy as np
from quest_teleop_core.quaternion_utils import multiply, rotate_vector

from .hand_model import synthetic_hand
from .hand_protocol import (LEFT, POS_TRACKED, POS_VALID, RIGHT, ROT_TRACKED, ROT_VALID, HandPacket,
                            encode_hand_packet)

ALL_OK = POS_VALID | ROT_VALID | POS_TRACKED | ROT_TRACKED
INDEX_JOINTS = range(6, 11)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5006)
    ap.add_argument("--hz", type=float, default=60.0)
    ap.add_argument("--dropout", action="store_true")
    ap.add_argument("--left-clutch", action="store_true")
    ap.add_argument("--clutch-period", type=float, default=20.0)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0, seq, left_seq, period = time.perf_counter(), 0, 0, 1.0 / args.hz
    left_local, left_quat = synthetic_hand([0.0] * 5)
    print(f"Sending fake right hand to udp://{args.host}:{args.port} @ {args.hz} Hz")

    try:
        while True:
            t = time.perf_counter() - t0

            # Wrist: slow circle in the OpenXR X-Z plane with a gentle yaw sway
            wrist_p = np.array([0.08 * math.cos(t), 1.2, -0.4 + 0.08 * math.sin(t)])
            yaw = 0.4 * math.sin(0.7 * t)
            wrist_q = np.array([0.0, math.sin(yaw / 2), 0.0, math.cos(yaw / 2)])

            # Fingers: curl wave travelling thumb -> little
            curls = [0.5 + 0.5 * math.sin(1.5 * t - 0.6 * i) for i in range(5)]
            local_p, local_q = synthetic_hand(curls)
            pos = np.array([wrist_p + rotate_vector(wrist_q, p) for p in local_p])
            quat = np.array([multiply(wrist_q, q) for q in local_q])
            flags = np.full(26, ALL_OK, dtype=np.uint8)

            if args.dropout and (t % 6.0) < 0.5:
                for i in INDEX_JOINTS:
                    flags[i], pos[i] = 0, 0.0  # lost joints arrive as zeros, like the real device

            pkt = HandPacket(RIGHT, True, seq, t, pos, quat, flags)
            sock.sendto(encode_hand_packet(pkt), (args.host, args.port))
            seq += 1

            if args.left_clutch:
                closed = (t % args.clutch_period) < 0.75 * args.clutch_period
                lpos = left_local + np.array([-0.2, 1.2, -0.35])
                lpos[5] = lpos[10] + np.array([0.015 if closed else 0.09, 0.0, 0.0])  # thumb tip vs index tip
                lflags = np.full(26, ALL_OK, dtype=np.uint8)
                sock.sendto(encode_hand_packet(HandPacket(LEFT, True, left_seq, t, lpos, left_quat, lflags)),
                            (args.host, args.port))
                left_seq += 1
            time.sleep(period)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
