#!/usr/bin/env python3
"""
Sanity-check the combined robot description. Needs `xacro` on PATH and `ur_description`
discoverable (source your ROS 2 workspace first).

    python3 tools/check_description.py
"""

import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "ros2_ws/src/quest_teleop_description"


def main() -> int:
    xml = subprocess.run(
        ["xacro", str(PKG / "urdf/ur5e_shadow.urdf.xacro"), "ur_type:=ur5e"],
        check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(xml)

    links = [l.get("name") for l in root.findall("link")]
    joints = root.findall("joint")
    moving = [j for j in joints if j.get("type") not in ("fixed",)]
    hand = [j.get("name") for j in moving if j.get("name").startswith("hand_")]
    arm = [j.get("name") for j in moving if not j.get("name").startswith("hand_")]
    children = {j.find("child").get("link") for j in joints}
    roots = [l for l in links if l not in children]
    missing = []
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename", "")
        prefix = "package://quest_teleop_description/"
        if uri.startswith(prefix) and not (PKG / uri[len(prefix):]).exists():
            missing.append(uri)

    checks = {
        "single kinematic root 'world'": roots == ["world"],
        "6 arm joints": len(arm) == 6,
        "22 movable hand joints": len(hand) == 22,
        "unique link names": len(links) == len(set(links)),
        "unique joint names": len({j.get("name") for j in joints}) == len(joints),
        "hand mounted on tool0": any(j.get("name") == "hand_mount" and j.find("parent").get("link") == "tool0"
                                     for j in joints),
        "2 ros2_control systems": len(root.findall("ros2_control")) == 2,
        "hand meshes all exist": not missing,
    }
    for name, ok in checks.items():
        print(("PASS  " if ok else "FAIL  ") + name)
    if missing:
        print("missing meshes:", missing[:5])
    print(f"\narm joints: {arm}\nhand joints: {len(hand)}, links total: {len(links)}")
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
