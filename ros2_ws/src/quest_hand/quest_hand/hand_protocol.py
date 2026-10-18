"""
Binary wire format for Quest hand-tracking packets (ROS-independent).

One UDP datagram = one hand, one frame. Little-endian, fixed size (774 bytes,
well under a typical 1472-byte UDP payload so it is never fragmented).

    Header (20 B):  magic "QHND" | version u8 | hand u8 | flags u8 | n_joints u8
                    | seq u32 | device_time f64 (seconds)
    26 x Joint (29 B): px py pz (f32 x3) | qx qy qz qw (f32 x4) | joint_flags u8

Poses are in the OpenXR tracking space (metres, right-handed, Y-up).
The Unity/C# sender must implement exactly this layout - see docs/hand_protocol.md.
"""

import struct
from dataclasses import dataclass

import numpy as np

MAGIC = b"QHND"
VERSION = 1
NUM_JOINTS = 26
LEFT, RIGHT = 0, 1

# Packet-level flags
HAND_TRACKED = 1 << 0

# Per-joint flags (mirror XrSpaceLocationFlags)
POS_VALID = 1 << 0
ROT_VALID = 1 << 1
POS_TRACKED = 1 << 2
ROT_TRACKED = 1 << 3

PALM, WRIST = 0, 1

_HEADER = struct.Struct("<4sBBBBId")
_JOINT = struct.Struct("<7fB")
PACKET_SIZE = _HEADER.size + NUM_JOINTS * _JOINT.size  # 774

JOINT_NAMES = [
    "palm", "wrist",
    "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip",
    "index_metacarpal", "index_proximal", "index_intermediate", "index_distal", "index_tip",
    "middle_metacarpal", "middle_proximal", "middle_intermediate", "middle_distal", "middle_tip",
    "ring_metacarpal", "ring_proximal", "ring_intermediate", "ring_distal", "ring_tip",
    "little_metacarpal", "little_proximal", "little_intermediate", "little_distal", "little_tip",
]

# Joint index pairs drawn as bones in RViz
BONES = (
    [(1, 2), (2, 3), (3, 4), (4, 5)]
    + [(1, 6), (6, 7), (7, 8), (8, 9), (9, 10)]
    + [(1, 11), (11, 12), (12, 13), (13, 14), (14, 15)]
    + [(1, 16), (16, 17), (17, 18), (18, 19), (19, 20)]
    + [(1, 21), (21, 22), (22, 23), (23, 24), (24, 25)]
    + [(7, 12), (12, 17), (17, 22)]  # knuckle line
)


class ProtocolError(ValueError):
    """Raised for a datagram that is not a valid hand packet."""


@dataclass
class HandPacket:
    hand: int                 # LEFT or RIGHT
    tracked: bool
    seq: int
    device_time: float
    positions: np.ndarray     # (26, 3)
    quats: np.ndarray         # (26, 4)  [x, y, z, w]
    flags: np.ndarray         # (26,) uint8


def encode_hand_packet(pkt: HandPacket) -> bytes:
    out = [_HEADER.pack(MAGIC, VERSION, pkt.hand, HAND_TRACKED if pkt.tracked else 0,
                        NUM_JOINTS, pkt.seq & 0xFFFFFFFF, pkt.device_time)]
    for p, q, f in zip(pkt.positions, pkt.quats, pkt.flags):
        out.append(_JOINT.pack(p[0], p[1], p[2], q[0], q[1], q[2], q[3], int(f)))
    return b"".join(out)


def decode_hand_packet(data: bytes) -> HandPacket:
    """Parse and sanitise one datagram. Raises ProtocolError if it is unusable."""
    if len(data) != PACKET_SIZE:
        raise ProtocolError(f"expected {PACKET_SIZE} bytes, got {len(data)}")
    magic, version, hand, pflags, n_joints, seq, t = _HEADER.unpack_from(data, 0)
    if magic != MAGIC:
        raise ProtocolError("bad magic")
    if version != VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    if n_joints != NUM_JOINTS:
        raise ProtocolError(f"expected {NUM_JOINTS} joints, got {n_joints}")
    if hand not in (LEFT, RIGHT):
        raise ProtocolError(f"bad hand id {hand}")
    if not np.isfinite(t):
        raise ProtocolError("non-finite device time")

    rows = np.array(list(_JOINT.iter_unpack(data[_HEADER.size:])), dtype=float)
    positions, quats = rows[:, 0:3].copy(), rows[:, 3:7].copy()
    flags = rows[:, 7].astype(np.uint8)

    # Sanitise rather than reject: a few bad joints should not drop the frame.
    bad_pos = ~np.isfinite(positions).all(axis=1)
    positions[bad_pos] = 0.0
    flags[bad_pos] &= np.uint8(~POS_VALID & 0xFF)

    norms = np.linalg.norm(np.nan_to_num(quats), axis=1)
    bad_rot = ~np.isfinite(quats).all(axis=1) | (norms < 1e-6)
    quats[bad_rot] = [0.0, 0.0, 0.0, 1.0]
    flags[bad_rot] &= np.uint8(~ROT_VALID & 0xFF)
    quats[~bad_rot] /= norms[~bad_rot, None]

    return HandPacket(hand, bool(pflags & HAND_TRACKED), seq, float(t), positions, quats, flags)
