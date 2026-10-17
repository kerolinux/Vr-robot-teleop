#!/usr/bin/env python3
"""
fake_quest_sender.py

Stand-in for the Quest app so the ROS side can be developed and tested
without a headset. Sends a hand moving in a circle at ~90 Hz (Quest tracking
rate) with a pinch that toggles every 2 seconds.

Usage: ros2 run quest_bridge fake_quest_sender --ros-args  (no ROS needed) or
       python3 fake_quest_sender.py [--host 127.0.0.1] [--port 5005] [--hz 90]
"""

import argparse
import json
import math
import socket
import time


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--hz", type=float, default=90.0)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.hz
    t0 = time.perf_counter()
    print(f"Sending fake hand poses to udp://{args.host}:{args.port} @ {args.hz} Hz")

    try:
        while True:
            t = time.perf_counter() - t0
            # Circle in the OpenXR X-Z plane (Y is up), ~1.2 m above floor
            packet = {
                "pos": [0.15 * math.cos(t), 1.2, -0.4 + 0.15 * math.sin(t)],
                "quat": [0.0, 0.0, 0.0, 1.0],
                "pinch": 1.0 if int(t / 2) % 2 else 0.0,
            }
            sock.sendto(json.dumps(packet).encode("utf-8"), (args.host, args.port))
            time.sleep(period)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
