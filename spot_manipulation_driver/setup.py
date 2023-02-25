import os
from glob import glob

from setuptools import setup

package_name = "spot_manipulation_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, "scripts"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="spot",
    maintainer_email="jpanthi@utexas.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "follow_joint_trajectory_action_server = scripts.follow_joint_trajectory_action_server:main"
        ]
    },
)
