"""
Left-hand fingers for the two-arm setup: conditions the left skeleton (mirrored, because the robot hand model is a
right hand) and retargets it onto the left Shadow Hand. Also gives RViz a left skeleton.

Expects: /quest/left_hand/skeleton_raw (from udp_hand_receiver), the left hand controller
/left_hand_position_controller, and the left arm's engage topic /left/teleop/engaged.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="quest_hand", executable="hand_conditioner", name="left_hand_conditioner",
             output="screen",
             parameters=[{
                 "hand": "left",
                 "input_topic": "/quest/left_hand/skeleton_raw",
                 "output_topic": "/teleop/left_hand/keypoints",
                 "marker_topic": "/teleop/left_hand/markers",
                 "viz_origin": [0.4, 0.5, 0.4],       # mirror image of the right skeleton's spot (y = -0.5)
                 "mirror_y": True,
             }]),
        Node(package="quest_teleop_control", executable="hand_retarget", name="left_hand_retarget",
             output="screen",
             parameters=[{
                 "input_topic": "/teleop/left_hand/keypoints",
                 "output_topic": "/left_hand_position_controller/commands",
                 "engaged_topic": "/left/teleop/engaged",
             }]),
    ])
