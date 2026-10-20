"""Everything for the simulated teleoperation chain (run the fake senders separately)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(pkg, file, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(pkg), "launch", file])),
        launch_arguments=(args or {}).items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        _include("quest_bridge", "bridge.launch.py"),
        _include("quest_hand", "hand_bridge.launch.py"),
        _include("quest_teleop_core", "teleop_core.launch.py",
                 {"debug_tf": "false", "use_robot_feedback": "true"}),
        _include("quest_teleop_control", "retarget.launch.py"),
        _include("quest_teleop_control", "control.launch.py",
                 {"launch_rviz": LaunchConfiguration("launch_rviz")}),
    ])
