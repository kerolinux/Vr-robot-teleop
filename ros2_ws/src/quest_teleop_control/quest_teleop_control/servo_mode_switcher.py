#!/usr/bin/env python3
"""
servo_mode_switcher.py

MoveIt Servo (Jazzy) ignores every command until it is told which command type to accept.
This node waits for Servo's `switch_command_type` service and selects POSE (2), so that the
PoseStamped stream on /teleop/target_pose is tracked. It retries until Servo accepts, then
keeps running quietly (so it can be re-launched with the rest of the stack).

    ServoCommandType: JOINT_JOG = 0, TWIST = 1, POSE = 2
Manual equivalent:
    ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 2}"
"""

import rclpy
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node


class ServoModeSwitcher(Node):
    def __init__(self) -> None:
        super().__init__("servo_mode_switcher")
        self.declare_parameter("service", "/servo_node/switch_command_type")
        self.declare_parameter("command_type", ServoCommandType.Request.POSE)
        self._command_type = int(self.get_parameter("command_type").value)
        self._client = self.create_client(ServoCommandType, self.get_parameter("service").value)
        self._done = False
        self._pending = None
        self.create_timer(1.0, self._tick)
        self.get_logger().info("Waiting for MoveIt Servo ...")

    def _tick(self) -> None:
        if self._done or self._pending is not None:
            return
        if not self._client.service_is_ready():
            return
        req = ServoCommandType.Request()
        req.command_type = self._command_type
        self._pending = self._client.call_async(req)
        self._pending.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        self._pending = None
        try:
            ok = future.result().success
        except Exception as exc:  # noqa: BLE001 - service died mid-call; just retry
            self.get_logger().warn(f"switch_command_type failed: {exc}")
            return
        if ok:
            self._done = True
            self.get_logger().info("Servo is now accepting POSE commands on /teleop/target_pose")
        else:
            self.get_logger().warn("Servo refused the command type; will retry")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoModeSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
