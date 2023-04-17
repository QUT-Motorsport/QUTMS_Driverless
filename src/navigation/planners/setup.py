from setuptools import setup

package_name = "planners"

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
    maintainer="Alastair Bradford, Zac Gillerat",
    maintainer_email="team@gqutmotorsport.com",
    description="Path Planner for mapped tracks",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "delaunay_planner = planners.node_delaunay_planner:main",
            "ordered_mid_spline = planners.node_ordered_mid_spline:main",
            "local_planner = planners.node_local_planner:main",
            "midline_planner = planners.node_midline_planner:main",
            "cone_interpolator = planners.node_cone_interpolator:main",
        ],
    },
)
