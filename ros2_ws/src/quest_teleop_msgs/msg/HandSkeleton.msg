# Full hand skeleton (26 OpenXR hand-tracking joints).
#
# Joint order is the XR_EXT_hand_tracking order (see docs/hand_protocol.md):
#   0 PALM, 1 WRIST, 2-5 THUMB, 6-10 INDEX, 11-15 MIDDLE, 16-20 RING, 21-25 LITTLE
#
# raw topics:         poses in the OpenXR tracking frame
# conditioned topics: poses in the wrist frame with ROS axes
#                     (+X toward fingertips, +Y toward thumb side, +Z out of the back of the hand)
std_msgs/Header header
string hand                    # "left" or "right"
bool tracked                   # false = do not trust this frame
uint32 seq                     # sender sequence number (detects dropped packets)
float64 device_time            # Quest-side timestamp in seconds; only differences are meaningful
geometry_msgs/Pose[] joints    # always 26 entries
uint8[] flags                  # per joint: bit0 = position valid/fresh, bit1 = orientation valid
