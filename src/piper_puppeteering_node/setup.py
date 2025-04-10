from setuptools import find_packages, setup

package_name = "piper_puppeteering_node"

setup(
    name=package_name,
    version="0.0.0",
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
        # Define entry point wrapper to enable usage in Conda environments:
        (
            "lib/" + package_name,
            ["scripts/run_piper_puppeteering_node"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jscholz",
    maintainer_email="jscholz@reimaginerobotics.ai",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "piper_puppeteering_node = piper_puppeteering_node.puppeteering_node:main",
        ],
    },
)
