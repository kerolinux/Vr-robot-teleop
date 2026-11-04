#!/usr/bin/env python3
"""Check both supported two-arm descriptions (with and without the left hand).

Needs xacro and the sourced ROS workspace. Does not launch controllers.
"""
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "ros2_ws/src/quest_teleop_description"


def check_description(left_hand: bool) -> bool:
    xml = subprocess.run(
        ["xacro", str(PKG / "urdf/ur5e_shadow.urdf.xacro"), "ur_type:=ur5e",
         f"left_hand:={str(left_hand).lower()}"],
        check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(xml)
    links = [link.get("name") for link in root.findall("link")]
    joints = root.findall("joint")
    moving = {j.get("name") for j in joints if j.get("type") != "fixed"}
    children = {j.find("child").get("link") for j in joints}
    roots = [name for name in links if name not in children]
    arm_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}
    right_hand = {name for name in moving if name.startswith("hand_")}
    left_fingers = {name for name in moving if name.startswith("left_hand_")}
    expected_left = {"left_" + name for name in right_hand} if left_hand else set()
    systems = root.findall("ros2_control")
    control_joints = [j.get("name") for system in systems for j in system.findall("joint")]
    missing = []
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename", "")
        prefix = "package://quest_teleop_description/"
        if uri.startswith(prefix) and not (PKG / uri[len(prefix):]).exists():
            missing.append(uri)

    def mounted(name, parent):
        return any(j.get("name") == name and j.find("parent").get("link") == parent for j in joints)

    checks = {
        "single kinematic root world": roots == ["world"],
        "six named joints per arm": moving - right_hand - left_fingers ==
            arm_names | {"left_" + name for name in arm_names},
        "22 right hand joints": len(right_hand) == 22,
        "left hand joints match selected variant": left_fingers == expected_left,
        "unique link names": len(links) == len(set(links)),
        "unique joint names": len({j.get("name") for j in joints}) == len(joints),
        "right hand mounted on tool0": mounted("hand_mount", "tool0"),
        "left hand mount matches selected variant": mounted("left_hand_mount", "left_tool0") == left_hand,
        "hardware system count": len(systems) == (4 if left_hand else 3),
        "every movable joint controlled exactly once":
            len(control_joints) == len(set(control_joints)) and set(control_joints) == moving,
        "hand meshes exist": not missing,
    }
    print(f"\nleft_hand={left_hand}")
    for name, ok in checks.items():
        print(("PASS  " if ok else "FAIL  ") + name)
    if missing:
        print("Missing meshes:", missing)
    return all(checks.values())


def main() -> int:
    results = [check_description(left_hand) for left_hand in (False, True)]
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
