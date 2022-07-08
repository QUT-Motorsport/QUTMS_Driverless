from setuptools import setup

package_name = "missions"

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
    description="Mission control node to start driving events",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["mission_control = missions.node_mission_control:main", "gui = missions.node_gui:main"],
    },
)
