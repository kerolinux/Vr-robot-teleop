#!/usr/bin/env python3
"""
replay_node.py  (Phase 4)

Plays a recorded demonstration back through the same interfaces the live operator uses:
the arm target goes to /teleop/target_pose (MoveIt Servo tracks it) and the hand joint targets to
/hand_position_controller/commands. If the replayed motion matches the original, the dataset is a
faithful record of what the operator did.

    ros2 run quest_teleop_data replay_episode --ros-args -p file:=/path/session.hdf5 -p demo:=demo_0

Release the operator clutch (stop the clutch publisher) first: teleop_core and hand_retarget only
publish while engaged, so they will not fight the replay. The first action is held for
`settle_time` seconds so the arm can travel to the starting pose before playback begins.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .dataset_io import list_episodes, read_episode


class ReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("episode_replay")
        d = self.declare_parameter
        d("file", "")
        d("demo", "demo_0")
        d("speed", 1.0)
        d("settle_time", 4.0)
        d("loop", False)
        d("rate", 50.0)
        d("base_frame", "base_link")
        d("target_pose_topic", "/teleop/target_pose")
        d("hand_command_topic", "/hand_position_controller/commands")
        get = lambda n: self.get_parameter(n).value  # noqa: E731

        path, demo = get("file"), str(get("demo"))
        if not path:
            raise SystemExit("set the dataset with:  --ros-args -p file:=/path/to/session.hdf5")
        if demo.isdigit():
            demo = f"demo_{demo}"
        if demo not in list_episodes(path):
            raise SystemExit(f"{demo} not in {path}; available: {list_episodes(path)}")
        arrays, attrs = read_episode(path, demo)
        self._t = arrays["timestamps"] - arrays["timestamps"][0]
        self._actions = arrays["actions"].astype(float)
        self._speed, self._settle, self._loop = float(get("speed")), float(get("settle_time")), bool(get("loop"))
        self._frame = get("base_frame")

        self._pose_pub = self.create_publisher(PoseStamped, get("target_pose_topic"), 10)
        self._hand_pub = self.create_publisher(Float64MultiArray, get("hand_command_topic"), 10)
        self._start = self.get_clock().now()
        self._done = False
        self.create_timer(1.0 / float(get("rate")), self._tick)
        self.get_logger().info(
            f"Replaying {demo} ({len(self._t)} samples, {self._t[-1]:.1f} s, success={attrs['success']}) "
            f"after a {self._settle:.0f} s settle")

    def _publish(self, action: np.ndarray) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, action[0:3])
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = map(float, action[3:7])
        self._pose_pub.publish(msg)
        self._hand_pub.publish(Float64MultiArray(data=[float(x) for x in action[7:]]))

    def _tick(self) -> None:
        if self._done:
            return
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        if elapsed < self._settle:
            self._publish(self._actions[0])
            return
        t = (elapsed - self._settle) * self._speed
        if t >= self._t[-1]:
            self._publish(self._actions[-1])
            if self._loop:
                self._start = self.get_clock().now()
            else:
                self._done = True
                self.get_logger().info("Replay finished")
            return
        self._publish(self._actions[int(np.searchsorted(self._t, t, side="right")) - 1])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReplayNode()
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
