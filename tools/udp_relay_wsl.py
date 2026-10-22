#!/usr/bin/env python3
"""
udp_relay_wsl.py - forward the Quest's UDP packets from Windows into WSL2 (NAT mode).

Run on WINDOWS (plain Python 3, no packages):   python tools\\udp_relay_wsl.py

In NAT mode the headset can reach the Windows PC but not the WSL2 virtual machine behind it. This listens on
the Windows side and forwards every datagram to the WSL2 address, which it discovers itself with
`wsl.exe hostname -I` and re-checks every few seconds, because that address changes after `wsl --shutdown`
or a reboot. Point the Unity app's Host at the Windows PC's LAN IP; ROS (inside WSL) keeps listening on 5006.

    --listen-port 5006   port the headset sends to (Windows firewall must allow UDP on it)
    --target-port 5006   port udp_hand_receiver listens on inside WSL
    --target IP          skip discovery and always forward to this address
"""

import argparse
import socket
import subprocess
import threading
import time


def discover_wsl_ip() -> str:
    """First IPv4 address reported by the default WSL distribution, or '' if WSL is not running."""
    try:
        out = subprocess.run(["wsl.exe", "hostname", "-I"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ""
    for token in out.replace("\x00", "").split():
        if token.count(".") == 3:
            return token
    return ""


class Relay:
    def __init__(self, listen_port: int, target_port: int, fixed_target: str = "", host: str = "0.0.0.0"):
        self.target_port, self.fixed_target = target_port, fixed_target
        self.target = fixed_target
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind((host, listen_port))
        self.sock.settimeout(0.5)
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stop = threading.Event()
        self.forwarded = self.errors = self.no_target = 0

    def refresh_target(self, every: float = 5.0) -> None:
        while not self.stop.is_set():
            if not self.fixed_target:
                ip = discover_wsl_ip()
                if ip and ip != self.target:
                    print(f"[relay] WSL address is now {ip}")
                self.target = ip or self.target
            self.stop.wait(every)

    def run(self, report_every: float = 2.0) -> None:
        threading.Thread(target=self.refresh_target, daemon=True).start()
        last = time.time()
        while not self.stop.is_set():
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                data = None
            except OSError:
                break
            if data:
                if not self.target:
                    self.no_target += 1
                else:
                    try:
                        self.out.sendto(data, (self.target, self.target_port))
                        self.forwarded += 1
                    except OSError:
                        self.errors += 1
            if time.time() - last >= report_every:
                rate = self.forwarded / (time.time() - last)
                note = "" if self.target else "  (WSL not found: start an Ubuntu terminal)"
                print(f"[relay] {rate:6.1f} pkt/s -> {self.target or '?'}:{self.target_port}  "
                      f"errors={self.errors} dropped_no_target={self.no_target}{note}")
                self.forwarded = 0
                last = time.time()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen-port", type=int, default=5006)
    ap.add_argument("--target-port", type=int, default=5006)
    ap.add_argument("--target", default="")
    args = ap.parse_args()
    relay = Relay(args.listen_port, args.target_port, args.target)
    print(f"[relay] listening on UDP {args.listen_port}; Ctrl+C to stop")
    try:
        relay.run()
    except KeyboardInterrupt:
        pass
    finally:
        relay.stop.set()


if __name__ == "__main__":
    main()
