#!/usr/bin/env python3
"""
Generate urdf/shadow_hand.xacro from the community URDF (AndrejOrsula/shadow_hand_ign).

Transformations applied to the upstream file:
  * wrapped in a xacro macro with a `prefix` and an attachment (`parent` + origin) joint
  * every link / joint name gets ${prefix}
  * mesh URIs point at this package's vendored copy of the meshes
  * the two wrist joints (forearm_joint, wrist_joint) become FIXED: the hand is mounted on a
    6-DOF arm, so the extra wrist DOF would only add redundancy -> 22 movable hand joints
  * typo fix: index_finger_join2 -> index_finger_joint2
  * a second macro declares mock ros2_control position interfaces for the 22 joints

Usage: python3 tools/generate_shadow_hand_xacro.py /path/to/shadow_hand_ign \
           ros2_ws/src/quest_teleop_description/urdf/shadow_hand.xacro
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PKG = "quest_teleop_description"
FIXED = {"forearm_joint", "wrist_joint"}
RENAMES = {"index_finger_join2": "index_finger_joint2"}


def main(upstream: Path, out: Path) -> None:
    root = ET.parse(upstream / "urdf" / "shadow_hand.urdf").getroot()
    unexpected = {c.tag for c in root} - {"link", "joint"}
    if unexpected:
        raise SystemExit(f"Unexpected top-level URDF tags: {unexpected}")

    movable = []  # (name, lower, upper)
    body = []
    for el in root:
        if el.tag == "link":
            el.set("name", "${prefix}" + el.get("name"))
            for mesh in el.iter("mesh"):
                mesh.set("filename", mesh.get("filename").replace(
                    "package://shadow_hand_ign/meshes/", f"package://{PKG}/meshes/shadow_hand/"))
        else:
            name = RENAMES.get(el.get("name"), el.get("name"))
            for tag in ("parent", "child"):
                el.find(tag).set("link", "${prefix}" + el.find(tag).get("link"))
            if name in FIXED:
                el.set("type", "fixed")
                for tag in ("axis", "limit", "dynamics"):
                    for child in el.findall(tag):
                        el.remove(child)
            else:
                lim = el.find("limit")
                movable.append((name, float(lim.get("lower")), float(lim.get("upper"))))
            el.set("name", "${prefix}" + name)
        ET.indent(el, space="    ", level=2)
        body.append("    " + ET.tostring(el, encoding="unicode").rstrip())

    ctrl = []
    for name, lo, hi in movable:
        ctrl.append(f"""      <joint name="${{prefix}}{name}">
        <command_interface name="position">
          <param name="min">{lo:.6f}</param>
          <param name="max">{hi:.6f}</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
      </joint>""")

    text = f"""<?xml version="1.0"?>
<!--
  GENERATED FILE - do not edit by hand. Regenerate with tools/generate_shadow_hand_xacro.py
  Source: https://github.com/AndrejOrsula/shadow_hand_ign (BSD), visual meshes/kinematics from
  shadow-robot/sr_common. See THIRD_PARTY_NOTICES.md.
  Wrist joints are fixed: {len(movable)} movable hand joints.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Shadow Hand (no wrist DOF) mounted on `parent` at the given origin. -->
  <xacro:macro name="shadow_hand" params="prefix parent *origin">
    <joint name="${{prefix}}mount" type="fixed">
      <parent link="${{parent}}"/>
      <child link="${{prefix}}forearm"/>
      <xacro:insert_block name="origin"/>
    </joint>

""" + "\n\n".join(body) + f"""
  </xacro:macro>

  <!-- Mock ros2_control hardware (position command interfaces) for the hand joints. -->
  <xacro:macro name="shadow_hand_ros2_control" params="name prefix">
    <ros2_control name="${{name}}" type="system">
      <hardware>
        <plugin>mock_components/GenericSystem</plugin>
      </hardware>
""" + "\n".join(ctrl) + """
    </ros2_control>
  </xacro:macro>

</robot>
"""
    out.write_text(text)
    print(f"wrote {out}  ({len(movable)} movable joints)")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    main(Path(sys.argv[1]), Path(sys.argv[2]))
