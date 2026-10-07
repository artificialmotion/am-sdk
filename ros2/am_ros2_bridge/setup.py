# SPDX-License-Identifier: Apache-2.0
import os
from glob import glob

from setuptools import setup

package_name = "am_ros2_bridge"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Artificial Motion",
    maintainer_email="ayo@artificialmotion.com",
    description="Python-only ROS 2 bridge for Artificial Motion joints on the am-sdk simulator (SIMULATED).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "am_bridge = am_ros2_bridge.bridge_node:main",
        ],
    },
)
