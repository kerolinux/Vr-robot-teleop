"""Left arm chain: left wrist pose -> second teleop_core (namespace 'left') -> /left/teleop/target_pose."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    core_yaml = os.path.join(get_package_share_directory("quest_teleop_core"), "config", "teleop.yaml")
    with open(core_yaml) as f:
        core = yaml.safe_load(f)["teleop_core"]["ros__parameters"]
    core.update({
        "input_pose_topic": "/quest/left_hand/pose_raw",
        "input_pinch_topic": "/quest/left_hand/pinch",
        "input_clutch_topic": "/quest/clutch",          # shared left-pinch clutch for now
        "target_pose_topic": "/left/teleop/target_pose",
        "gripper_topic": "/left/teleop/gripper_cmd",
        "engaged_topic": "/left/teleop/engaged",
        "base_frame": "left_base_link",
        "ee_frame": "left_tool0",
        "use_robot_feedback": True,
        "clutch_source": "always",
    })
    return LaunchDescription([
        Node(package="quest_hand", executable="hand_inputs", name="left_hand_inputs", output="screen",
             parameters=[{
                 "publish_pose": True,
                 "pose_hand": "left",
                 "pose_topic": "/quest/left_hand/pose_raw",
                 "pinch_topic": "/quest/left_hand/pinch",
                 "clutch_hand": "right",                    # its clutch output is unused,
                 "clutch_topic": "/quest/unused_clutch",    # so it must not touch /quest/clutch
             }]),
        Node(package="quest_teleop_core", executable="teleop_core", name="teleop_core",
             namespace="left", output="screen", parameters=[core]),
    ])
