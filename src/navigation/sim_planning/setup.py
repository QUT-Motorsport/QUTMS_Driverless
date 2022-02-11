from setuptools import setup

package_name = "sim_planning"

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
    description="Plans and follows an ideal path through the track",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mapper = sim_planning.node_map:main",
            "pursuit = sim_planning.node_pursuit:main"
        ],
    },
)
