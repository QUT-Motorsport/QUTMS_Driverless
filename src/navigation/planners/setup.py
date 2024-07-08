from glob import glob
import os

from setuptools import setup

package_name = "planners"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alastair Bradford, Zac Gillerat, Damin Guerin",
    maintainer_email="team@qutmotorsport.com",
    description="Path Planner for mapped tracks",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ordered_mid_spline_node = planners.node_ordered_mid_spline:main",
            "ordered_mid_spline_lifecycle = planners.lifecycle_ordered_mid_spline:main",
            "boundary_interpolation_node = planners.node_boundary_interpolation:main",
            "ft_planner_node = planners.node_ft_planner:main",
        ],
    },
)
