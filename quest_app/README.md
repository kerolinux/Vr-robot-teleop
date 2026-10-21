# Quest app: hand-tracking telemetry sender

A Unity script that streams Meta Quest hand tracking (26 joints per hand) over UDP to the ROS 2
machine, using the packet format in [`docs/hand_protocol.md`](../docs/hand_protocol.md).

```
Quest 2 --OpenXR hand tracking--> Unity XR Hands --HandTelemetrySender--> UDP :5006 --> ROS 2
```

Only two scripts matter, both in `Assets/Scripts/QuestTeleop/`:

| File | Purpose |
|---|---|
| `HandPacketEncoder.cs` | Builds the 774-byte packet and converts Unity poses to the OpenXR frame. No Unity dependencies, so it is tested with plain .NET. |
| `HandTelemetrySender.cs` | MonoBehaviour: finds the XR Hands subsystem, sends one packet per hand per frame. |

## What was verified, and what was not

- **Verified (automatically):** the C# encoder's bytes decode to the same values in the Python reference decoder
  (`python3 tools/verify_unity_packet.py`, needs the .NET SDK), and the Unity-to-OpenXR rotation conversion matches
  explicit rotation matrices. `HandTelemetrySender.cs` compiles against stand-ins for the Unity types built from
  the XR Hands documentation.
- **Not verified:** the script has not been run in Unity or on a headset. The Unity and Quest settings below are
  the standard route as documented by Unity and Meta; menu names move between Unity versions, so treat them as a
  checklist and adapt. If something differs, the first thing to check is the XR Hands **HandVisualizer** sample
  (step 5): if it tracks your hands, the sender will too.

## 1. Create the project

Unity 2022.3 LTS or Unity 6, with the **Android Build Support** module (including OpenJDK and Android SDK & NDK
Tools) installed through Unity Hub. In Unity Hub, create a project from the **VR** template if your version offers
one (it already contains an XR Origin and OpenXR setup); otherwise use a 3D template and add them yourself.
Copy `Assets/Scripts/QuestTeleop/` from this folder into the project's `Assets/`.

## 2. Packages

Window > Package Manager > Unity Registry. Install **XR Plug-in Management**, **OpenXR Plugin** and **XR Hands**
(`com.unity.xr.hands`). The sender uses `XRHandSubsystem.updatedHands`, `XRHand.GetJoint`, `XRHandJoint.TryGetPose`
and `XRHandJointID`, which exist across the 1.x releases.

## 3. XR settings

Edit > Project Settings > XR Plug-in Management, **Android tab**: tick **OpenXR**. Then under OpenXR (Android tab):

- Enable **Meta Quest Support** (in newer versions this comes from the *Unity OpenXR: Meta* package, which the
  Package Manager offers when you enable it).
- Enable the **Hand Tracking Subsystem** feature (this is what provides `XRHandSubsystem`).

## 4. Player settings (Android)

Edit > Project Settings > Player > Android > Other Settings:

- Scripting Backend: **IL2CPP**; Target Architectures: **ARM64**.
- Minimum API Level: **Android 10 (API 29)** or higher.
- **Internet Access: Require.** Without it the app cannot send UDP.

## 5. Check that hands track before using the sender

Package Manager > XR Hands > Samples > import **HandVisualizer**, open its scene, build and run it on the Quest.
You should see two hands following yours. Do this first: if it does not work, the problem is in the XR setup
(steps 1-4), not in the sender. On the Quest itself, make sure Settings > Movement tracking > **Hand tracking** is
on, and put the controllers down so it switches to hands.

## 6. Add the sender

Add `HandTelemetrySender` to any GameObject in a scene that runs XR. Set **Host** to the IP of the PC running ROS
(on Windows: `ipconfig`, the IPv4 address of the adapter you use; with the WSL2 *mirrored* networking from the
setup notes, WSL shares that same address). The Quest and the PC must be on the same network. Keep **Port** at
5006 unless you change it in `quest_hand/config/hand.yaml`.

Build and run: File > Build Profiles (or Build Settings) > Android > Switch Platform; put the Quest in developer
mode, connect it by USB, and choose Run Device. The PC firewall must allow UDP 5006 (see below).

## 7. PC side

Allow the port once, in an **admin PowerShell** (same pattern as the earlier port 5005 rules; check the exact
commands against Microsoft's WSL networking documentation):

```powershell
New-NetFirewallRule -DisplayName "Quest hand UDP 5006" -Direction Inbound -Protocol UDP -LocalPort 5006 -Action Allow
New-NetFirewallHyperVRule -Name "QuestHandUDP" -DisplayName "Quest hand UDP 5006" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP -LocalPorts 5006
```

Then, in Ubuntu (workspace sourced):

```bash
ros2 launch quest_teleop_control teleop_sim.launch.py real_headset:=true
```

`real_headset:=true` makes the arm follow your right wrist from the skeleton stream. **Do not** run the Phase 1
`fake_quest_sender` with it. Check that data arrives:

```bash
ros2 topic hz /quest/right_hand/skeleton_raw     # about 60 Hz
ros2 topic hz /quest/right_hand/pose_raw          # about 60 Hz
ros2 topic echo /quest/clutch                     # 0.0 normally, up to 1.0 while pinching
```

## Using it

- **Left hand: pinch thumb and index finger = engage.** Release = the arm stops and holds. Losing tracking of
  that hand, or a network stall, also releases it.
- **Right hand: wrist motion** drives the arm; **right fingers** drive the robot fingers.
- The arm moves relative to where your hand was when you engaged, so you can release, reposition your arm and
  engage again without the robot jumping.
- Face the direction you want to count as "toward the robot" when the app starts (the Quest's tracking origin is
  set then), or set `base_yaw_deg` in `quest_teleop_core/config/teleop.yaml`.
- Pinch thresholds are in `quest_hand/config/hand.yaml` (`pinch_open_dist`, `pinch_closed_dist`).

## Testing the whole path without a headset

```bash
ros2 launch quest_teleop_control teleop_sim.launch.py real_headset:=true
ros2 run quest_hand fake_hand_sender --left-clutch
```

The fake sender streams a right hand (wrist circling, fingers waving) and a left hand whose pinch closes and
opens every 20 s, so the arm engages, moves and releases exactly as it would with the headset.

## Troubleshooting

| Symptom | Check |
|---|---|
| Nothing arrives on `skeleton_raw` | Host IP correct? Same network? Firewall rule for UDP 5006? `adb logcat -s Unity` shows the sender's once-a-second status line. |
| Status says right/left `LOST` | Hands not visible to the headset cameras; controllers still held; hand tracking off in Quest settings. |
| `skeleton_raw` works but the arm never moves | Is the left pinch reaching 1.0 on `/quest/clutch`? Tune `pinch_open_dist` / `pinch_closed_dist`. |
| Arm moves in the wrong direction | Set `base_yaw_deg` (rotation about the vertical axis), or recentre the headset. |
| Fingers bend the wrong way | Retargeting sign conventions are inferred; see `config/retarget.yaml` and report which joint. |
| `XRHandSubsystem` not found at compile time | The XR Hands package is not installed, or the scripts sit in an assembly definition that does not reference `Unity.XR.Hands`. |
