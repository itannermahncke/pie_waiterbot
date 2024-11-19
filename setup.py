import os
from setuptools import find_packages, setup
from glob import glob

package_name = "pie_waiterbot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ivymahncke",
    maintainer_email="imahncke@olin.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_adapter = pie_waiterbot.serial_adapter:main",
            "teleop_serial = pie_waiterbot.teleop_serial:main",
            "goal_driver = pie_waiterbot.goal_driver:main",
            "pose_estimation = pie_waiterbot.pose_estimation:main",
            "webcam_driver = pie_waiterbot.webcam_driver:main",
            "dead_reckoning = pie_waiterbot.dead_reckoning:main",
            "map_maker = pie_waiterbot.map_maker:main",
        ],
    },
)
