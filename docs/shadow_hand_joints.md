# Shadow Hand joints (22 movable, wrist fixed)

Joint names are those of the community model (`shadow_hand_ign`), prefixed `hand_` in the combined robot.
The *Shadow name* column is **inferred** from joint ranges and finger structure, not taken from an official
table: verify it against Shadow's documentation before relying on it for retargeting.

| Joint (without `hand_` prefix) | Shadow name (inferred) | Role | Lower (deg) | Upper (deg) |
|---|---|---|---|---|
| `thumb_joint1` | THJ5 | thumb | -60.0 | 60.0 |
| `thumb_joint2` | THJ4 | thumb | 0.0 | 70.0 |
| `thumb_joint3` | THJ3 | thumb | -12.0 | 12.0 |
| `thumb_joint4` | THJ2 | thumb | -40.0 | 40.0 |
| `thumb_joint5` | THJ1 | thumb | 0.0 | 90.0 |
| `index_finger_joint1` | FFJ4 | abduction | -20.0 | 20.0 |
| `index_finger_joint2` | FFJ3 | MCP flexion | 0.0 | 90.0 |
| `index_finger_joint3` | FFJ2 | PIP flexion | 0.0 | 90.0 |
| `index_finger_joint4` | FFJ1 | DIP flexion | 0.0 | 90.0 |
| `middle_finger_joint1` | MFJ4 | abduction | -20.0 | 20.0 |
| `middle_finger_joint2` | MFJ3 | MCP flexion | 0.0 | 90.0 |
| `middle_finger_joint3` | MFJ2 | PIP flexion | 0.0 | 90.0 |
| `middle_finger_joint4` | MFJ1 | DIP flexion | 0.0 | 90.0 |
| `ring_finger_joint1` | RFJ4 | abduction | -20.0 | 20.0 |
| `ring_finger_joint2` | RFJ3 | MCP flexion | 0.0 | 90.0 |
| `ring_finger_joint3` | RFJ2 | PIP flexion | 0.0 | 90.0 |
| `ring_finger_joint4` | RFJ1 | DIP flexion | 0.0 | 90.0 |
| `little_finger_joint1` | LFJ5 | palm arch | 0.0 | 45.0 |
| `little_finger_joint2` | LFJ4 | abduction | -20.0 | 20.0 |
| `little_finger_joint3` | LFJ3 | MCP flexion | 0.0 | 90.0 |
| `little_finger_joint4` | LFJ2 | PIP flexion | 0.0 | 90.0 |
| `little_finger_joint5` | LFJ1 | DIP flexion | 0.0 | 90.0 |

## Things that matter for retargeting

- **Distal joints are independent in this model.** On the real hand the last two joints of the index, middle,
  ring and little fingers are coupled mechanically (check Shadow's documentation), so a retargeter that commands
  them freely can ask for postures the hardware cannot reach. Decide in Phase 3b whether to enforce the coupling.
- **Hand frame.** In the model the forearm's +Z runs along the arm toward the palm and the thumb sits on +X.
  The conditioned Quest skeleton uses +X toward the fingertips, +Y toward the thumb, +Z out of the back of the
  hand, so Phase 3b needs one fixed rotation between the two.
- **Mass.** The upstream README assumes 4.2 kg total for the hand; compare that with the arm's rated payload.
- **Wrist.** `forearm_joint` and `wrist_joint` are fixed. They can be re-enabled by editing
  `tools/generate_shadow_hand_xacro.py` (`FIXED`) and regenerating.
