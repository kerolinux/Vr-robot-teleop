"""Static checks that keep the YAML configs and Python tables in agreement (no ROS needed)."""

from pathlib import Path

import yaml

from quest_teleop_control.hand_joints import HAND_JOINT_NAMES, HAND_JOINTS, HAND_PREFIX

CONFIG = Path(__file__).resolve().parents[1] / "config"


def _load(name):
    return yaml.safe_load((CONFIG / name).read_text())


def test_hand_controller_lists_exactly_the_hand_joints_in_order():
    joints = _load("controllers.yaml")["hand_position_controller"]["ros__parameters"]["joints"]
    assert joints == [HAND_PREFIX + n for n in HAND_JOINT_NAMES]
    assert len(joints) == 22


def test_arm_controller_has_the_six_ur_joints():
    joints = _load("controllers.yaml")["forward_position_controller"]["ros__parameters"]["joints"]
    assert joints == ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def test_all_declared_controllers_have_a_config_section():
    cfg = _load("controllers.yaml")
    declared = [k for k, v in cfg["controller_manager"]["ros__parameters"].items() if isinstance(v, dict)]
    assert set(declared) == {"joint_state_broadcaster", "forward_position_controller", "hand_position_controller"}
    for name in declared:
        if name != "joint_state_broadcaster":
            assert name in cfg


def test_servo_wiring_matches_the_rest_of_the_pipeline():
    servo = _load("servo.yaml")
    assert servo["move_group_name"] == "ur_manipulator"
    assert servo["planning_frame"] == "base_link"
    assert servo["command_out_topic"] == "/forward_position_controller/commands"
    assert servo["pose_command_in_topic"] == "/teleop/target_pose"   # teleop_core's output
    assert servo["is_primary_planning_scene_monitor"] is True        # no move_group in this bringup


def test_joint_limits_are_sane():
    for name, lo, hi in HAND_JOINTS:
        assert lo <= 0.0 <= hi, name  # every joint can start at 0 (open hand)
