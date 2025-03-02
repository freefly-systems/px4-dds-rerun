import os
from glob import glob

from setuptools import setup

package_name = "px4_ros_rerun"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/px4_ros_rerun_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Peter Breuer",
    maintainer_email="peter@freeflysystems.com",
    description="Package for visualizing PX4 drones with Rerun",
    license="BSD 3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "px4_ros_rerun = px4_ros_rerun.px4_ros_rerun:main",
        ],
    },
)
