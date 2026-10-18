"""Small helpers shared by the ROS nodes."""

from geometry_msgs.msg import Pose


def make_pose(p, q) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
    pose.orientation.x, pose.orientation.y = float(q[0]), float(q[1])
    pose.orientation.z, pose.orientation.w = float(q[2]), float(q[3])
    return pose
