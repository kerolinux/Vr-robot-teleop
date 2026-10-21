// Hand telemetry wire format, v1. The reference implementation is
// ros2_ws/src/quest_hand/quest_hand/hand_protocol.py and the layout is documented in
// docs/hand_protocol.md. This file has no Unity dependencies, so Tests/EncoderCheck compiles it with
// plain .NET and tools/verify_unity_packet.py checks its bytes against the Python decoder.

using System;

namespace QuestTeleop
{
    /// <summary>Serialises one hand-tracking frame into the 774-byte UDP datagram.</summary>
    public static class HandPacketEncoder
    {
        public const byte Version = 1;
        public const int NumJoints = 26;
        public const int HeaderSize = 20;   // "QHND", version, hand, flags, n_joints, seq (u32), time (f64)
        public const int JointSize = 29;    // 7 x f32 (position, quaternion xyzw) + 1 flag byte
        public const int PacketSize = HeaderSize + NumJoints * JointSize;

        public const byte Left = 0;
        public const byte Right = 1;

        public const byte HandTrackedFlag = 1 << 0;   // packet-level

        public const byte PosValid = 1 << 0;          // per joint (same meaning as XrSpaceLocationFlags)
        public const byte RotValid = 1 << 1;
        public const byte PosTracked = 1 << 2;
        public const byte RotTracked = 1 << 3;
        public const byte AllValid = PosValid | RotValid | PosTracked | RotTracked;

        /// <summary>
        /// Writes a packet into <paramref name="buffer"/> and returns its length (always PacketSize).
        /// positions: 26 x (x, y, z); rotations: 26 x (x, y, z, w); both in the OpenXR tracking frame
        /// (right-handed, +Y up, -Z forward). Use FrameConversion for Unity poses.
        /// </summary>
        public static int Encode(byte[] buffer, byte hand, bool tracked, uint seq, double deviceTime,
                                 float[] positions, float[] rotations, byte[] jointFlags)
        {
            if (buffer == null || buffer.Length < PacketSize)
                throw new ArgumentException("buffer must hold " + PacketSize + " bytes", nameof(buffer));
            if (positions == null || positions.Length != NumJoints * 3)
                throw new ArgumentException("positions must hold 26 x 3 floats", nameof(positions));
            if (rotations == null || rotations.Length != NumJoints * 4)
                throw new ArgumentException("rotations must hold 26 x 4 floats", nameof(rotations));
            if (jointFlags == null || jointFlags.Length != NumJoints)
                throw new ArgumentException("jointFlags must hold 26 bytes", nameof(jointFlags));

            int o = 0;
            buffer[o++] = (byte)'Q';
            buffer[o++] = (byte)'H';
            buffer[o++] = (byte)'N';
            buffer[o++] = (byte)'D';
            buffer[o++] = Version;
            buffer[o++] = hand;
            buffer[o++] = tracked ? HandTrackedFlag : (byte)0;
            buffer[o++] = NumJoints;
            o = PutU32(buffer, o, seq);
            o = PutF64(buffer, o, deviceTime);

            for (int k = 0; k < NumJoints; k++)
            {
                for (int j = 0; j < 3; j++) o = PutF32(buffer, o, positions[k * 3 + j]);
                for (int j = 0; j < 4; j++) o = PutF32(buffer, o, rotations[k * 4 + j]);
                buffer[o++] = jointFlags[k];
            }
            return o;
        }

        // Explicit little-endian writes: the wire format must not depend on the device's byte order.
        static int PutU32(byte[] b, int o, uint v)
        {
            b[o] = (byte)v;
            b[o + 1] = (byte)(v >> 8);
            b[o + 2] = (byte)(v >> 16);
            b[o + 3] = (byte)(v >> 24);
            return o + 4;
        }

        static int PutF32(byte[] b, int o, float f)
        {
            return PutU32(b, o, unchecked((uint)BitConverter.SingleToInt32Bits(f)));
        }

        static int PutF64(byte[] b, int o, double d)
        {
            ulong v = unchecked((ulong)BitConverter.DoubleToInt64Bits(d));
            for (int i = 0; i < 8; i++) b[o + i] = (byte)(v >> (8 * i));
            return o + 8;
        }
    }

    /// <summary>Unity (left-handed, +Z forward) to OpenXR (right-handed, -Z forward) conversion.</summary>
    public static class FrameConversion
    {
        // Unity's XR Hands API already converts OpenXR data into Unity's frame by mirroring Z.
        // Undoing it: positions flip z; a rotation R becomes S R S with S = diag(1, 1, -1), which for a
        // quaternion negates the x and y components (the rotation axis is an axial vector, so it picks up
        // an extra sign). tools/verify_unity_packet.py checks this against explicit rotation matrices.
        public static void PositionToOpenXr(float x, float y, float z, out float ox, out float oy, out float oz)
        {
            ox = x;
            oy = y;
            oz = -z;
        }

        public static void RotationToOpenXr(float qx, float qy, float qz, float qw,
                                            out float ox, out float oy, out float oz, out float ow)
        {
            ox = -qx;
            oy = -qy;
            oz = qz;
            ow = qw;
        }
    }
}
