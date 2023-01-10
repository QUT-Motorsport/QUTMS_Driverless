from setuptools import setup

package_name = "path_follower"

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
    description="Driverless Controllers",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pure_pursuit = path_follower.node_pure_pursuit:main",
        ],
    },
)
