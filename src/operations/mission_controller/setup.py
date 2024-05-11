from glob import glob
import os

from setuptools import setup

package_name = "mission_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alastair Bradford",
    maintainer_email="team@qutmotorsport.com",
    description="Mission control node to start driving events",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_launcher_node = mission_controller.node_mission_launcher:main",
            "inspection_handler_node = mission_controller.node_inspection_handler:main",
            "trackdrive_handler_node = mission_controller.node_trackdrive_handler:main",
            "trackdrive_handler_lifecycle = mission_controller.lifecycle_trackdrive_handler:main",
            "trackdrive_handler_nav_node = mission_controller.node_trackdrive_handler_nav:main",
        ],
    },
)
