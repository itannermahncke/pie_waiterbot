from setuptools import find_packages, setup

package_name = "pie_waiterbot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "teleop = pie_waiterbot.teleop:main",
            "serial_adapter = pie_waiterbot.serial_adapter:main",
            "teleop_serial = pie_waiterbot.teleop_serial:main",
        ],
    },
)
