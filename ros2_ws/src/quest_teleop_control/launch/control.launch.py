"""
UR5e + Shadow Hand with mock ros2_control hardware, MoveIt Servo and RViz.

    robot_state_publisher -> /robot_description
    ros2_control_node     -> joint_state_broadcaster, forward_position_controller (arm),
                             hand_position_controller (22 hand joints)
    servo_node            -> tracks /teleop/target_pose (PoseStamped, frame base_link)
    servo_mode_switcher   -> puts Servo into POSE mode
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")

    pkg = get_package_share_directory("quest_teleop_control")
    controllers_yaml = os.path.join(pkg, "config", "controllers.yaml")
    with open(os.path.join(pkg, "config", "servo.yaml")) as f:
        servo_params = {"moveit_servo": yaml.safe_load(f)}

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ",
                 PathJoinSubstitution([FindPackageShare("quest_teleop_description"), "urdf",
                                       "ur5e_shadow.urdf.xacro"]),
                 " ur_type:=", ur_type,
                 " left_hand:=", LaunchConfiguration("left_hand")]),
        value_type=str)

    # Kinematics, joint limits and SRDF come from UR's own (tested) MoveIt config package.
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .to_moveit_configs()
    )

    def spawner(name):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--controller-manager", "/controller_manager"])

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("left_hand", default_value="true"),

        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_description}]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[controllers_yaml]),
        spawner("joint_state_broadcaster"),
        spawner("forward_position_controller"),
        spawner("hand_position_controller"),
        spawner("left_forward_position_controller"),
        spawner("left_hand_position_controller"),

        # Give the controllers and /joint_states a moment before Servo starts reading the robot state.
        TimerAction(period=5.0, actions=[
            Node(package="moveit_servo", executable="servo_node", name="servo_node", output="screen",
                 parameters=[moveit_config.to_dict(), servo_params]),
            Node(package="quest_teleop_control", executable="servo_mode_switcher", output="screen"),
            Node(package="moveit_servo", executable="servo_node", name="servo_node", namespace="left", output="screen",
                 parameters=[moveit_config.to_dict(),
                             {"robot_description": robot_description,
                              "robot_description_semantic": open(os.path.join(pkg, "config", "left.srdf")).read()},
                             {"moveit_servo": yaml.safe_load(open(os.path.join(pkg, "config", "left_servo.yaml")))},
                             {"robot_description_kinematics": {"left_ur_manipulator": {
                                 "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                                 "kinematics_solver_search_resolution": 0.005,
                                 "kinematics_solver_timeout": 0.005}}},
                             {"robot_description_planning": {"joint_limits": {
                                 "left_" + j: {"has_velocity_limits": True, "max_velocity": 3.14,
                                               "has_acceleration_limits": True, "max_acceleration": 5.0}
                                 for j in ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]}}}]),
            Node(package="quest_teleop_control", executable="servo_mode_switcher", name="left_servo_mode_switcher", output="screen",
                 parameters=[{"service": "/left/servo_node/switch_command_type"}]),
        ]),

        Node(package="rviz2", executable="rviz2", output="screen", condition=IfCondition(launch_rviz),
             arguments=["-d", os.path.join(pkg, "rviz", "teleop.rviz")]),
    ])
