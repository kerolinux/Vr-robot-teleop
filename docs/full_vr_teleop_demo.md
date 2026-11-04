# Full VR teleoperation demonstration

This milestone demonstrates **two simulated UR5e arms with Shadow Hands controlled
by Meta Quest 2 optical hand tracking**. Both hands control arm position, wrist
twist/tilt and finger articulation. A shared keyboard clutch lets the operator
reposition their hands, and a mock-only reset recovers the arms from blocked poses.

This is a control demonstration in RViz with mock `ros2_control` hardware.
There are no objects, contact physics or trained policy yet. Reaching down and
closing fingers demonstrates the controls; it does not prove a successful grasp.
No demonstration video is currently included in this repository.

## Start the demonstration

Build the workspace first if you have just pulled changes. Run these commands in
**Ubuntu 24.04**, not PowerShell (`wsl -d Ubuntu-24.04` opens Ubuntu from PowerShell):

```bash
cd ~/Vr-robot-teleop/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

### Windows: relay

Run one relay in **Windows PowerShell**. For the repository under the current
WSL user's home directory:

```powershell
python -u "\\wsl.localhost\Ubuntu-24.04\home\<linux-user>\Vr-robot-teleop\tools\udp_relay_wsl.py"
```

Replace `<linux-user>` with your Ubuntu username. The relay discovers WSL's address.
Set the Quest application's Host to the **Windows PC's LAN IP**, port **5006**.
Quest and PC must be on the same network, with UDP 5006 allowed through the firewall.
Keep the custom Unity Quest application open with hand tracking active.
See [Quest setup](../quest_app/README.md) if it has not been installed.

### Ubuntu: four terminals

In **each** Ubuntu terminal:

```bash
cd ~/Vr-robot-teleop
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

Run one of the following commands per terminal:

**1: Robot, RViz, both Servo nodes, right arm and fingers**

```bash
ros2 launch quest_teleop_control teleop_sim.launch.py real_headset:=true track_orientation:=true
```

**2: Left arm**

```bash
ros2 launch quest_teleop_control left_teleop.launch.py track_orientation:=true
```

**3: Left fingers**

```bash
ros2 launch quest_teleop_control left_hand.launch.py
```

**4: Keyboard clutch and reset**

```bash
python3 tools/keyboard_clutch.py
```

Keep keyboard focus in terminal 4. Space toggles engagement, **0** releases,
**R** resets both mock arms, and **Q** exits with the clutch released.
Only one keyboard publisher should run. The status display reports each arm's
actual engagement, wrist freshness and Servo status. An engagement request alone
does not confirm that an arm is moving.

For both skeleton displays, load
`ros2_ws/install/quest_teleop_control/share/quest_teleop_control/rviz/teleop_two_arm.rviz`
in RViz. Both robot arms are present in the default robot model already.

## Demonstration walkthrough

1. **Ready posture:** show both open robot hands, fingers forward, palms down,
   thumbs inward. Place your real hands comfortably while released.
2. **Engage:** press Space. Move each hand separately to demonstrate independent
   right/left control.
3. **Position:** make small forward/back, sideways and up/down movements, one axis
   at a time. Repeat with both hands together.
4. **Orientation:** gently twist each wrist, then tilt the hand down and up while
   keeping its position roughly fixed. The wrist speed cap is 0.9 rad/s (~52 degrees/s);
   it can be lower near singularities or joint limits.
5. **Fingers:** open/close your hands and move individual fingers. Demonstrate a
   downward reach and finger closure without claiming object contact or grasping.
6. **Clutch:** press 0, reposition your hands and press Space. The robot should
   re-anchor without a target jump.
7. **Recovery:** press R. Both clutches release, Servo pauses and the mock arms
   return gradually to the ready joint pose. Wait for completion, reposition your
   hands and press Space again. Do not deliberately force an emergency stop.
8. **Finish:** press 0, then Q. Stop the launch processes with Ctrl+C when done.

These steps can also serve as a short screen-recording script. Show RViz and the
keyboard status together; a synchronized view of the operator's hands makes the
mapping clear. A visual recording and an HDF5 learning dataset are separate artifacts.

## Current mapping and recovery behavior

Both arms share `quest_teleop_core/config/teleop.yaml`:

- `invert_forward: true`, `invert_lateral: true`: invert base-frame X/Y displacement;
  vertical position retains its original mapping.
- `mirror_orientation: true`, `invert_tilt: true`: the accepted relative wrist mapping.
  Rotation is anchored at engagement, not snapped to absolute human-hand orientation.
- `track_orientation: true`: enable twist/tilt. False is a position-only diagnostic mode.

The mapping was accepted by the operator for the current view. It is not continuous
headset-POV alignment: the protocol does not transmit head pose. Avoid changing
working axes to compensate for an arbitrary RViz camera angle.

Tracking loss releases control and requires release/re-engagement after tracking
returns. Singularity stops remain enabled. Re-engagement alone does not move the
robot out of a singularity; R supplies a mock-only recovery path. Reset refuses
non-mock hardware and requires both teleop cores, Servo services and fresh joint
feedback. It finishes released. A reset failure may leave Servo paused; restart
the simulation before continuing. Reset motion is not collision planned.

## Validation and next milestone

Operator checks confirmed both-arm position directions, wrist twist and tilt,
finger response, hand placement and reset. The latest speed increase to 0.9 rad/s
passed simulation checks; its subjective response still needs operator confirmation.
The existing automated test runs in isolated localhost ROS domain 174:

```bash
python3 tools/smoke_teleop.py
```

It checks both arms' translation, 5 degrees twist and tilt, release/dropout behavior,
reset/re-anchoring and paired recording. It does not measure full-range orientation
reliability, optical tracking latency, physical robot safety or grasp success.

Next: build a physics scene with an object and a defined success condition, then
collect and inspect demonstrations before training a baseline policy. Paired HDF5
recording already exists; see [dataset format](dataset_format.md) and the
[recording commands](../README.md#recording-and-replaying-demonstrations).
