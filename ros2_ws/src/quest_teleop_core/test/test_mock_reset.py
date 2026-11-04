"""Offline checks for reset gating and hand reflection."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]

def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / (name + '.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_reset_rejects_real_and_mixed_hardware():
    mod = load('reset_mock_arms')
    def xml(plugins):
        return '<robot>' + ''.join('<ros2_control><hardware><plugin>' + p +
            '</plugin></hardware></ros2_control>' for p in plugins) + '</robot>'
    mock = 'mock_components/GenericSystem'
    assert mod.is_mock(xml([mock, mock]))
    assert not mod.is_mock(xml([mock, 'ur_robot_driver/URPositionHardwareInterface']))
    assert not mod.is_mock('<robot/>')


def test_reset_interpolation_has_no_overshoot():
    mod = load('reset_mock_arms')
    assert mod.interpolate([1., -2.], [0., 2.], -1.) == [1., -2.]
    assert mod.interpolate([1., -2.], [0., 2.], 2.) == [0., 2.]
    assert mod.interpolate([1., -2.], [0., 2.], .5) == [.5, 0.]


def test_hand_mount_forward_palms_down_thumbs_inward():
    root = ET.parse(ROOT / 'ros2_ws/src/quest_teleop_description/urdf/ur5e_shadow.urdf.xacro').getroot()
    ns = '{http://wiki.ros.org/xacro}'
    rpy = next(a.get('default') for a in root.findall(ns+'arg') if a.get('name') == 'hand_mount_rpy')
    tool = Rotation.from_quat([2**-.5, -2**-.5, 0, 0]).as_matrix()
    world = tool @ Rotation.from_euler('xyz', list(map(float,rpy.split()))).as_matrix()
    assert np.allclose(world @ [0,0,1], [1,0,0])  # fingers forward
    assert np.allclose(world @ [0,-1,0], [0,0,-1])  # flexion toward ground
    assert np.allclose(world @ [1,0,0], [0,1,0])  # right thumb inward
    left = next(a for a in root.iter(ns+'shadow_hand') if a.get('prefix')=='left_hand_')
    assert left.get('mirror') == '-1'
    assert np.allclose(world @ [-1,0,0], [0,-1,0])  # left thumb inward


def test_reflection_preserves_flexion_and_mirrors_abduction():
    mod = load('generate_shadow_hand_xacro')
    joint = ET.fromstring('<joint><origin xyz="1 2 3" rpy="0 0.5 0.2"/><axis xyz="1 0 0"/></joint>')
    mod.reflectable(joint)
    assert joint.find('axis').get('xyz') == '1 ${mirror * (0)} ${mirror * (0)}'
    assert joint.find('origin').get('xyz') == '${mirror * (1)} 2 3'


def test_reset_node_starts_without_commanding():
    import rclpy
    mod = load('reset_mock_arms')
    rclpy.init()
    node = mod.Reset()
    try:
        assert len(node.pause_clients) == 2
        assert node.released == {}
        import pytest
        with pytest.raises(RuntimeError, match='clutches'):
            node.check()
    finally:
        node.destroy_node()
        rclpy.shutdown()
