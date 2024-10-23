from setuptools import find_packages, setup

package_name = "slam_gridmap"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Claudia Kiss",
    maintainer_email="team@qutmotorsport.com",
    description="real-time SLAM cone detection operating on gridmaps",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["gridmap_to_cone_detection_node = slam_gridmap.node_gridmap_to_cone_detection:main"],
    },
)
