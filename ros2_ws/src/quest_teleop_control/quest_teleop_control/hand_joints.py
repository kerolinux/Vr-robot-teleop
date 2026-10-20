"""Shadow Hand joint list (without the `hand_` prefix) and limits, in controller order.

GENERATED from the expanded robot description. config/controllers.yaml lists the same joints
in the same order; test/test_config_consistency.py checks that they agree.
"""

HAND_PREFIX = "hand_"

# (name, lower [rad], upper [rad])
HAND_JOINTS = [
    ("thumb_joint1", -1.047198, 1.047198),
    ("thumb_joint2", 0.000000, 1.221730),
    ("thumb_joint3", -0.209440, 0.209440),
    ("thumb_joint4", -0.698132, 0.698132),
    ("thumb_joint5", 0.000000, 1.570796),
    ("index_finger_joint1", -0.349066, 0.349066),
    ("index_finger_joint2", 0.000000, 1.570796),
    ("index_finger_joint3", 0.000000, 1.570796),
    ("index_finger_joint4", 0.000000, 1.570796),
    ("middle_finger_joint1", -0.349066, 0.349066),
    ("middle_finger_joint2", 0.000000, 1.570796),
    ("middle_finger_joint3", 0.000000, 1.570796),
    ("middle_finger_joint4", 0.000000, 1.570796),
    ("ring_finger_joint1", -0.349066, 0.349066),
    ("ring_finger_joint2", 0.000000, 1.570796),
    ("ring_finger_joint3", 0.000000, 1.570796),
    ("ring_finger_joint4", 0.000000, 1.570796),
    ("little_finger_joint1", 0.000000, 0.785398),
    ("little_finger_joint2", -0.349066, 0.349066),
    ("little_finger_joint3", 0.000000, 1.570796),
    ("little_finger_joint4", 0.000000, 1.570796),
    ("little_finger_joint5", 0.000000, 1.570796),
]

HAND_JOINT_NAMES = [n for n, _, _ in HAND_JOINTS]
