from setuptools import find_packages, setup

package_name = "slam_gridmap"

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
    maintainer="yurikun1033",
    maintainer_email="claudiakiss005@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["gridmap_to_code_detection = slam_gridmap.node_gridmap_to_code_detection:main"],
    },
)
