"""Setup script for the piper_control_ros2 package."""

# Note: We install an executable wrapper script manually to work around ROS2`s
# default behavior of hardcoding `#!/usr/bin/python3` as the shebang in
# generated launch entrypoints. This causes Python packages installed only in
# Conda to be invisible at runtime.
#
# By installing a wrapper script into `lib/<package>`, and using `#!/usr/bin/env
# python` as its shebang, we ensure that ROS2 launches the node using the
# currently active environment's Python interpreter (e.g., from Conda).
#
# This allows us to `ros2 run piper_control_ros2 run_piper_control_node` while
# correctly using the Conda environment for dependencies.

import os

from setuptools import find_packages, setup

package_name = "piper_control_ros2"

setup(
    name=package_name,
    version="1.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "data"),
            ["piper_control_ros2/teach_mode/piper_grav_comp.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jscholz",
    maintainer_email="jscholz@reimaginerobotics.ai",
    description="A Python interface to the Piper robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "piper_control_node = piper_control_ros2.piper_control_node:main",
            "piper_mocked_node = piper_control_ros2.piper_mocked_node:main",
        ],
    },
)
