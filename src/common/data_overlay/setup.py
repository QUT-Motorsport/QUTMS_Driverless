from glob import glob
import os

from setuptools import setup

package_name = "data_overlay"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alastair Bradford",
    maintainer_email="team@qutmotorsport.com",
    description="Data overlay for videos",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "overlay = data_overlay.node_overlay:main",
        ],
    },
)
