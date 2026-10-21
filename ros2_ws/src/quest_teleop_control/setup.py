import os
from glob import glob

from setuptools import find_packages, setup

package_name = "quest_teleop_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.srdf")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Simulation bringup for Quest teleoperation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "servo_mode_switcher = quest_teleop_control.servo_mode_switcher:main",
            "hand_demo = quest_teleop_control.hand_demo:main",
            "hand_retarget = quest_teleop_control.hand_retarget_node:main",
        ],
    },
)
