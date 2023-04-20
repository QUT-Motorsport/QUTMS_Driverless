from glob import glob
import os

from setuptools import setup

package_name = "custom_vis"

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
    maintainer="Alistair English",
    maintainer_email="team@qutmotorsport.com",
    description="Custom visualisations",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "matrix = custom_vis.node_matrix_vis:main",
        ],
    },
)
