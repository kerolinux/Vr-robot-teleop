#!/usr/bin/env python3
"""Return BOTH mock arms to their configured starting pose with Servo paused.

Invoked by keyboard_clutch's R key. Never use for physical hardware.
A failed reset leaves Servo paused; restart the simulation to recover.
"""
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String, Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

NAMES = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
PREFIXES = ('', 'left_')


def is_mock(description):
    root = ET.fromstring(description)
    systems = root.findall('ros2_control')
    return len(systems) >= 2 and all(
        s.findtext('hardware/plugin') == 'mock_components/GenericSystem' for s in systems)


def interpolate(start, target, fraction):
    t = max(0., min(1., fraction))
    t = t * t * (3. - 2. * t)
    return [a + t * (b - a) for a, b in zip(start, target)]


class Reset(Node):
    def __init__(self):
        super().__init__('reset_mock_arms')
        self.description = None
        self.released = {}
        self.joints = {}
        self.joint_time = 0.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.description_cb, latched)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, qos_profile_sensor_data)
        for prefix in ('', '/left'):
            self.create_subscription(Bool, prefix + '/teleop/engaged',
                lambda m, p=prefix: self.released.update({p: not m.data}), latched)
        self.pause_clients = [self.create_client(SetBool, p + '/servo_node/pause_servo')
                        for p in ('', '/left')]
        self.pubs = [self.create_publisher(Float64MultiArray,
            '/' + p + 'forward_position_controller/commands', 10) for p in PREFIXES]

    def description_cb(self, msg):
        self.description = msg.data

    def joint_cb(self, msg):
        self.joints = dict(zip(msg.name, msg.position))
        self.joint_time = time.monotonic()

    def wait_for(self, predicate, timeout=5.):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError('Timed out waiting for mock robot / released clutches / Servo response')
            rclpy.spin_once(self, timeout_sec=.02)

    def check(self):
        if not all(self.released.get(p, False) for p in ('', '/left')):
            raise RuntimeError('Both clutches must stay released')
        if not all(math.isfinite(self.joints.get(p+n, float('nan'))) for p in PREFIXES for n in NAMES):
            raise RuntimeError('Joint positions are missing or non-finite')
        if time.monotonic() - self.joint_time > .25:
            raise RuntimeError('Joint feedback is stale')

    def pause(self, value):
        for client in self.pause_clients:
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError('Both Servo pause services are required')
            future = client.call_async(SetBool.Request(data=value))
            self.wait_for(future.done)
            if not future.result().success:
                raise RuntimeError('Servo refused pause/resume')

    def run(self):
        self.wait_for(lambda: self.description is not None and
            all(self.released.get(p, False) for p in ('', '/left')) and
            all(p + n in self.joints for p in PREFIXES for n in NAMES))
        if not is_mock(self.description):
            raise RuntimeError('Reset refused: every hardware system must be mock_components/GenericSystem')
        self.check()
        self.pause(True)
        # Drain any commands already queued before pause acknowledgement.
        deadline = time.monotonic() + .3
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
        self.check()
        start = [[self.joints[p + n] for n in NAMES] for p in PREFIXES]
        config = Path(__file__).resolve().parents[1] / 'ros2_ws/src/quest_teleop_description/config/initial_positions.yaml'
        values = yaml.safe_load(config.read_text())
        target = [float(values[n]) for n in NAMES]
        # Smoothstep peak speed <= 0.3 rad/s. This path is mock-only, not collision planned.
        duration = max(3., 5. * max(abs(a-b) for arm in start for a,b in zip(arm,target)))
        began = time.monotonic()
        while True:
            rclpy.spin_once(self, timeout_sec=.02)
            self.check()
            fraction = (time.monotonic() - began) / duration
            for pub, arm in zip(self.pubs, start):
                pub.publish(Float64MultiArray(data=interpolate(arm, target, fraction)))
            if fraction >= 1.:
                break
            time.sleep(.02)
        self.wait_for(lambda: all(abs(self.joints[p+n]-v) < .01
            for p in PREFIXES for n,v in zip(NAMES,target)), timeout=3.)
        self.check()
        self.pause(False)
        print('Mock arms reset. Clutch remains released; place your hands comfortably, then press SPACE.')


def main():
    rclpy.init()
    node = Reset()
    try:
        node.run()
        return 0
    except Exception as exc:
        print(f'Reset failed: {exc}. Servo may remain paused; restart the simulation before continuing.')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
