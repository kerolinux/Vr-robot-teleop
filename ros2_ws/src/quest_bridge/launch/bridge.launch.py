from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="quest_bridge",
            executable="udp_pose_receiver",
            name="udp_pose_receiver",
            output="screen",
            parameters=[{
                "host": "0.0.0.0",
                "port": 5005,
                "frame_id": "quest_world",
            }],
        ),
    ])
