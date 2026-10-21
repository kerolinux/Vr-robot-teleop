// Streams Meta Quest hand tracking (26 joints per hand, via Unity XR Hands + OpenXR) to the ROS 2
// machine as UDP datagrams in the format defined in docs/hand_protocol.md.
//
// Add this component to any GameObject in a scene that has XR running (see quest_app/README.md).

using System;
using System.Collections.Generic;
using System.Net.Sockets;
using UnityEngine;
using UnityEngine.XR.Hands;

namespace QuestTeleop
{
    public class HandTelemetrySender : MonoBehaviour
    {
        [Header("Network")]
        [Tooltip("IP address of the PC running ROS 2 (with WSL2 mirrored networking: the Windows LAN IP).")]
        public string host = "192.168.1.100";
        public int port = 5006;

        [Header("What to send")]
        public bool sendRightHand = true;   // drives the arm (wrist pose) and the robot fingers
        public bool sendLeftHand = true;    // thumb-index pinch of this hand is the clutch
        [Tooltip("Upper limit on packets per second per hand. Quest hand tracking runs at about 60 Hz.")]
        public float maxRateHz = 60f;

        /// <summary>One-line summary for debugging (e.g. assign to a UI text).</summary>
        public string Status { get; private set; } = "starting";

        // Protocol joint order (OpenXR XR_EXT_hand_tracking): palm first, then wrist. Mapped by *name*, so
        // it does not depend on the numeric order of XRHandJointID.
        static readonly XRHandJointID[] JointOrder =
        {
            XRHandJointID.Palm, XRHandJointID.Wrist,
            XRHandJointID.ThumbMetacarpal, XRHandJointID.ThumbProximal, XRHandJointID.ThumbDistal, XRHandJointID.ThumbTip,
            XRHandJointID.IndexMetacarpal, XRHandJointID.IndexProximal, XRHandJointID.IndexIntermediate,
            XRHandJointID.IndexDistal, XRHandJointID.IndexTip,
            XRHandJointID.MiddleMetacarpal, XRHandJointID.MiddleProximal, XRHandJointID.MiddleIntermediate,
            XRHandJointID.MiddleDistal, XRHandJointID.MiddleTip,
            XRHandJointID.RingMetacarpal, XRHandJointID.RingProximal, XRHandJointID.RingIntermediate,
            XRHandJointID.RingDistal, XRHandJointID.RingTip,
            XRHandJointID.LittleMetacarpal, XRHandJointID.LittleProximal, XRHandJointID.LittleIntermediate,
            XRHandJointID.LittleDistal, XRHandJointID.LittleTip,
        };

        XRHandSubsystem _subsystem;
        UdpClient _udp;

        readonly byte[] _buffer = new byte[HandPacketEncoder.PacketSize];
        readonly float[] _positions = new float[HandPacketEncoder.NumJoints * 3];
        readonly float[] _rotations = new float[HandPacketEncoder.NumJoints * 4];
        readonly byte[] _flags = new byte[HandPacketEncoder.NumJoints];
        readonly uint[] _seq = new uint[2];
        readonly double[] _lastSend = new double[2];

        int _sent;
        int _errors;
        double _windowStart;

        void OnEnable()
        {
            try
            {
                _udp = new UdpClient();
                _udp.Connect(host, port);
                Status = "sending to " + host + ":" + port + " (waiting for hand tracking)";
            }
            catch (Exception e)
            {
                Status = "network setup failed: " + e.Message;
                Debug.LogError("HandTelemetrySender: " + Status);
                enabled = false;
            }
        }

        void OnDisable()
        {
            if (_subsystem != null)
            {
                _subsystem.updatedHands -= OnUpdatedHands;
                _subsystem = null;
            }
            if (_udp != null)
            {
                _udp.Close();
                _udp = null;
            }
        }

        void Update()
        {
            // The hand subsystem may not exist yet when the scene starts, so keep looking for it.
            if (_subsystem == null)
            {
                var found = new List<XRHandSubsystem>();
                SubsystemManager.GetSubsystems<XRHandSubsystem>(found);
                if (found.Count > 0)
                {
                    _subsystem = found[0];
                    _subsystem.updatedHands += OnUpdatedHands;
                    Debug.Log("HandTelemetrySender: hand subsystem found, streaming to " + host + ":" + port);
                }
            }

            double now = Time.realtimeSinceStartupAsDouble;
            if (now - _windowStart >= 1.0)
            {
                if (_subsystem != null)
                {
                    Status = string.Format("{0} pkt/s to {1}:{2}; right {3}, left {4}; send errors {5}",
                        _sent, host, port,
                        _subsystem.rightHand.isTracked ? "tracked" : "LOST",
                        _subsystem.leftHand.isTracked ? "tracked" : "LOST", _errors);
                    Debug.Log("HandTelemetrySender: " + Status);
                }
                _sent = 0;
                _windowStart = now;
            }
        }

        void OnUpdatedHands(XRHandSubsystem subsystem, XRHandSubsystem.UpdateSuccessFlags successFlags,
                            XRHandSubsystem.UpdateType updateType)
        {
            // The subsystem reports twice per frame; one packet per frame is enough.
            if (updateType != XRHandSubsystem.UpdateType.Dynamic) return;
            if (sendRightHand) SendHand(subsystem.rightHand, HandPacketEncoder.Right);
            if (sendLeftHand) SendHand(subsystem.leftHand, HandPacketEncoder.Left);
        }

        void SendHand(XRHand hand, byte handId)
        {
            double now = Time.realtimeSinceStartupAsDouble;
            if (maxRateHz > 0f && now - _lastSend[handId] < 1.0 / maxRateHz) return;
            _lastSend[handId] = now;

            bool tracked = hand.isTracked;
            for (int k = 0; k < HandPacketEncoder.NumJoints; k++)
            {
                bool ok = false;
                Pose pose = default(Pose);
                if (tracked)
                {
                    XRHandJoint joint = hand.GetJoint(JointOrder[k]);
                    ok = joint.TryGetPose(out pose);
                }

                if (ok)
                {
                    float px, py, pz, qx, qy, qz, qw;
                    FrameConversion.PositionToOpenXr(pose.position.x, pose.position.y, pose.position.z,
                                                     out px, out py, out pz);
                    FrameConversion.RotationToOpenXr(pose.rotation.x, pose.rotation.y, pose.rotation.z,
                                                     pose.rotation.w, out qx, out qy, out qz, out qw);
                    _positions[k * 3] = px; _positions[k * 3 + 1] = py; _positions[k * 3 + 2] = pz;
                    _rotations[k * 4] = qx; _rotations[k * 4 + 1] = qy;
                    _rotations[k * 4 + 2] = qz; _rotations[k * 4 + 3] = qw;
                    _flags[k] = HandPacketEncoder.AllValid;
                }
                else
                {
                    // Untracked joint: zeros with an identity rotation, flagged invalid. The ROS side holds the
                    // last good value for such joints instead of using these numbers.
                    _positions[k * 3] = _positions[k * 3 + 1] = _positions[k * 3 + 2] = 0f;
                    _rotations[k * 4] = _rotations[k * 4 + 1] = _rotations[k * 4 + 2] = 0f;
                    _rotations[k * 4 + 3] = 1f;
                    _flags[k] = 0;
                }
            }

            int length = HandPacketEncoder.Encode(_buffer, handId, tracked, _seq[handId]++, now,
                                                  _positions, _rotations, _flags);
            try
            {
                _udp.Send(_buffer, length);
                _sent++;
            }
            catch (Exception e) when (e is SocketException || e is ObjectDisposedException)
            {
                _errors++;
            }
        }
    }
}
