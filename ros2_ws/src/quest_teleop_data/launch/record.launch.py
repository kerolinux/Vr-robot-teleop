import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory("quest_teleop_data"), "config", "recorder.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="", description="Where datasets are written."),
        DeclareLaunchArgument("task", default_value="teleop_demo"),
        DeclareLaunchArgument("record_rosbag", default_value="false"),
        Node(package="quest_teleop_data", executable="episode_recorder", name="episode_recorder",
             output="screen",
             parameters=[params, {
                 "output_dir": LaunchConfiguration("output_dir"),
                 "task": LaunchConfiguration("task"),
                 "record_rosbag": ParameterValue(LaunchConfiguration("record_rosbag"), value_type=bool)}]),
    ])
