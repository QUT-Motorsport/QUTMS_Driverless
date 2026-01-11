from setuptools import find_packages, setup

package_name = "rosbag_creator"

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
    maintainer="Walden Leung",
    maintainer_email="team@qutmotorsport.com",
    description="Wraps rosbag as services to start and stop bags",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["rosbag_creator_node = rosbag_creator.node_rosbag_creator:main"],
    },
)
