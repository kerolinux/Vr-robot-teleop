import numpy as np
import pytest

from quest_hand.hand_model import synthetic_hand
from quest_hand.hand_protocol import (PACKET_SIZE, POS_VALID, RIGHT, ROT_VALID, HandPacket,
                                      ProtocolError, decode_hand_packet, encode_hand_packet)


def make_packet():
    pos, quat = synthetic_hand([0.3, 0.5, 0.5, 0.5, 0.5])
    return HandPacket(RIGHT, True, 42, 12.5, pos, quat, np.full(26, 0xF, dtype=np.uint8))


def test_packet_size_is_fixed_and_fits_in_one_datagram():
    assert PACKET_SIZE == 774
    assert len(encode_hand_packet(make_packet())) == PACKET_SIZE


def test_round_trip():
    src = make_packet()
    out = decode_hand_packet(encode_hand_packet(src))
    assert (out.hand, out.tracked, out.seq, out.device_time) == (RIGHT, True, 42, 12.5)
    assert np.allclose(out.positions, src.positions, atol=1e-6)  # float32 on the wire
    assert np.allclose(out.quats, src.quats, atol=1e-6)
    assert (out.flags == src.flags).all()


@pytest.mark.parametrize("mutate", [
    lambda b: b[:-1],                       # truncated
    lambda b: b"XXXX" + b[4:],              # bad magic
    lambda b: b[:4] + bytes([99]) + b[5:],  # unknown version
    lambda b: b[:5] + bytes([7]) + b[6:],   # bad hand id
])
def test_bad_packets_are_rejected(mutate):
    with pytest.raises(ProtocolError):
        decode_hand_packet(mutate(encode_hand_packet(make_packet())))


def test_non_finite_joint_is_sanitised_not_fatal():
    pkt = make_packet()
    pkt.positions[8] = np.nan
    pkt.quats[9] = 0.0  # zero quaternion
    out = decode_hand_packet(encode_hand_packet(pkt))
    assert not out.flags[8] & POS_VALID and np.all(out.positions[8] == 0)
    assert not out.flags[9] & ROT_VALID and np.allclose(out.quats[9], [0, 0, 0, 1])
    assert np.allclose(np.linalg.norm(out.quats, axis=1), 1.0)
