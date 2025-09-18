"""Setup script for the piper_puppeteering_node package."""

from setuptools import find_packages, setup

package_name = "piper_puppeteering_node"

setup(
    name=package_name,
    version="1.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/puppeteering_launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jscholz",
    maintainer_email="jscholz@reimaginerobotics.ai",
    description="Puppeteering node for the Piper robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "piper_puppeteering_node = piper_puppeteering_node.puppeteering_node:main",  # pylint: disable=line-too-long
        ],
    },
)
