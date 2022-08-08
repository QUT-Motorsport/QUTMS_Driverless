from setuptools import setup

package_name = "lidar_pipeline_2"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Liam Ferrante",
    maintainer_email="qutmotorsport.team@gmail.com",
    description="Real-time LiDAR perception pipeline for autonomous formula student racecar.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["lidar_perception = lidar_pipeline_2.node_lidar_perception:main"],
    },
)
