# Hand telemetry protocol (v1)

One UDP datagram per hand per frame, sent from the Quest app to `udp://<PC-IP>:5006`.
Little-endian, fixed size **774 bytes**. Reference implementation: `quest_hand/hand_protocol.py`.

## Header (20 bytes)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | char[4] | magic | `"QHND"` |
| 4 | u8 | version | `1` |
| 5 | u8 | hand | `0` = left, `1` = right |
| 6 | u8 | flags | bit0 = hand tracked |
| 7 | u8 | n_joints | always `26` |
| 8 | u32 | seq | increments per packet, wraps; lets the receiver count losses |
| 12 | f64 | device_time | seconds, Quest clock; only differences are used |

## Joints (26 x 29 bytes, starting at offset 20)

| Type | Field |
|---|---|
| f32 x3 | position x, y, z (metres) |
| f32 x4 | quaternion x, y, z, w |
| u8 | joint flags: bit0 position valid, bit1 orientation valid, bit2 position tracked, bit3 orientation tracked (same meaning as `XrSpaceLocationFlags`) |

Poses are in the OpenXR tracking space: right-handed, +Y up, -Z forward.
Send zeros for joints whose flags are 0.

## Joint order (`XR_EXT_hand_tracking`)

0 palm, 1 wrist, 2-5 thumb (metacarpal, proximal, distal, tip),
6-10 index, 11-15 middle, 16-20 ring, 21-25 little
(each: metacarpal, proximal, intermediate, distal, tip).

## Rate

Send at the tracking rate (60-72 Hz). Never retransmit: a lost frame is
replaced by the next one.
