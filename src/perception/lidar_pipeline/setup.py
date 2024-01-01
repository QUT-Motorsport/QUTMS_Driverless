from glob import glob
import os

from setuptools import setup

package_name = "lidar_pipeline"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Liam Ferrante",
    maintainer_email="team@qutmotorsport.com",
    description="Real-time LiDAR perception pipeline for autonomous formula student racecar.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar_debug_node = lidar_pipeline.node_lidar_debug:main",
            "lidar_detector_node = lidar_pipeline.node_lidar_detector:main",
        ],
    },
)
