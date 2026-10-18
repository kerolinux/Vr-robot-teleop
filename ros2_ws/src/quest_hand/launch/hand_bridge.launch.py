import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("quest_hand"), "config", "hand.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "debug_tf", default_value="false",
            description="Publish a static world->base_link TF. Leave false if teleop_core.launch.py "
                        "is already running (it publishes the same transform)."),
        Node(package="quest_hand", executable="udp_hand_receiver", name="udp_hand_receiver",
             output="screen", parameters=[params]),
        Node(package="quest_hand", executable="hand_conditioner", name="hand_conditioner",
             output="screen", parameters=[params]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="debug_world_to_base_hand",
             arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0",
                        "--yaw", "0", "--frame-id", "world", "--child-frame-id", "base_link"],
             condition=IfCondition(LaunchConfiguration("debug_tf"))),
    ])
