// Writes one deterministic packet to the file given as argument 0. tools/verify_unity_packet.py
// decodes it with the Python reference decoder and compares every field.

using System;
using System.IO;
using QuestTeleop;

internal static class Program
{
    private static int Main(string[] args)
    {
        if (args.Length != 1)
        {
            Console.Error.WriteLine("usage: EncoderCheck <output.bin>");
            return 2;
        }

        var positions = new float[HandPacketEncoder.NumJoints * 3];
        var rotations = new float[HandPacketEncoder.NumJoints * 4];
        var flags = new byte[HandPacketEncoder.NumJoints];
        for (int k = 0; k < HandPacketEncoder.NumJoints; k++)
        {
            for (int j = 0; j < 3; j++) positions[k * 3 + j] = 0.01f * (k + 1) + 0.001f * j;
            double theta = 0.05 * (k + 1);                       // rotation about +Y
            rotations[k * 4 + 1] = (float)Math.Sin(theta / 2);
            rotations[k * 4 + 3] = (float)Math.Cos(theta / 2);
            flags[k] = (k % 3 == 0) ? (byte)0 : HandPacketEncoder.AllValid;
        }

        var buffer = new byte[HandPacketEncoder.PacketSize];
        int length = HandPacketEncoder.Encode(buffer, HandPacketEncoder.Right, true, 123456u, 1234.5678,
                                              positions, rotations, flags);
        if (length != HandPacketEncoder.PacketSize)
        {
            Console.Error.WriteLine("unexpected packet length " + length);
            return 1;
        }
        File.WriteAllBytes(args[0], buffer);
        Console.WriteLine("wrote " + length + " bytes to " + args[0]);
        return 0;
    }
}
