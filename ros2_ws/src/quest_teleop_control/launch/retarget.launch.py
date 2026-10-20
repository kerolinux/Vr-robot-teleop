import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory("quest_teleop_control"), "config", "retarget.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("require_engaged", default_value="true",
                              description="Only move the fingers while the arm clutch is engaged."),
        Node(package="quest_teleop_control", executable="hand_retarget", name="hand_retarget",
             output="screen",
             parameters=[params, {"require_engaged": ParameterValue(
                 LaunchConfiguration("require_engaged"), value_type=bool)}]),
    ])
