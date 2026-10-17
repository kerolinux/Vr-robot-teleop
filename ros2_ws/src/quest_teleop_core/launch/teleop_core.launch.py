import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("quest_teleop_core"), "config", "teleop.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "debug_tf", default_value="true",
            description="Publish a static world->base_link TF so RViz can show the target. "
                        "Set false once the robot description provides base_link (Phase 3)."),
        Node(package="quest_teleop_core", executable="teleop_core", name="teleop_core",
             output="screen", parameters=[params]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="debug_world_to_base",
             arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0",
                        "--yaw", "0", "--frame-id", "world", "--child-frame-id", "base_link"],
             condition=IfCondition(LaunchConfiguration("debug_tf"))),
    ])
