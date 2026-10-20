import os
from glob import glob

from setuptools import find_packages, setup

package_name = "quest_teleop_data"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "h5py"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Demonstration recording for Quest teleoperation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "episode_recorder = quest_teleop_data.episode_recorder_node:main",
            "replay_episode = quest_teleop_data.replay_node:main",
            "inspect_dataset = quest_teleop_data.inspect_dataset:main",
        ],
    },
)
