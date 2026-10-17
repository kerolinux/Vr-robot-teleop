import os
from glob import glob

from setuptools import find_packages, setup

package_name = "quest_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Quest hand-tracking bridge for ROS 2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "udp_pose_receiver = quest_bridge.udp_pose_receiver:main",
            "fake_quest_sender = quest_bridge.fake_quest_sender:main",
        ],
    },
)
