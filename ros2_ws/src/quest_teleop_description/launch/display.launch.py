"""View the UR5e + Shadow Hand in RViz with sliders for every joint (no controllers needed)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("quest_teleop_description")
    xacro_file = PathJoinSubstitution([pkg, "urdf", "ur5e_shadow.urdf.xacro"])
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file, " ur_type:=", LaunchConfiguration("ur_type")]),
        value_type=str,
    )
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", PathJoinSubstitution([pkg, "rviz", "robot.rviz"])]),
    ])
