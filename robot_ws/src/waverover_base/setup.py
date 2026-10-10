from setuptools import find_packages, setup
import os
from glob import glob

package_name = "waverover_base"

setup(
    name=package_name,
    version="0.0.4",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="WAVE ROVER base driver, feedback, display battery percentage, and keyboard teleop.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "waverover_driver = waverover_base.waverover_driver:main",
            "keyboard_teleop = waverover_base.keyboard_teleop:main",
        ],
    },
)
