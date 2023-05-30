from glob import glob
import os

from setuptools import setup

package_name = "py_slam"

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
    description="Python approach to SLAM with known car position",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_slam = py_slam.node_imu_slam:main",
            "sbg_slam = py_slam.node_sbg_slam:main",
            "wss_slam = py_slam.node_wss_slam:main",
            "odom_slam = py_slam.node_odom_slam:main",
            "track_to_csv = py_slam.node_track_to_csv:main",
        ],
    },
)
