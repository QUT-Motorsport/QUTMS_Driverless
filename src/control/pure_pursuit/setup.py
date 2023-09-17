from setuptools import setup

package_name = "pure_pursuit"

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
    maintainer="Alastair Bradford, Grant van Breda, Damin Guerin, Zac Gillerat",
    maintainer_email="team@qutmotorsport.com",
    description="Driverless Guidance Systems",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pure_pursuit_node = pure_pursuit.node_pure_pursuit:main",
            "pure_pursuit_lifecycle = pure_pursuit.lifecycle_pure_pursuit:main",
            "particle_pursuit_node = pure_pursuit.node_particle_pursuit:main",
            "particle_pursuit_lifecycle = pure_pursuit.lifecycle_particle_pursuit:main",
        ],
    },
)
