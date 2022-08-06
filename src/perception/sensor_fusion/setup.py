from setuptools import setup

package_name = "sensor_fusion"

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
    author="Ian Rist",
    author_email="ian@bigair.net",
    maintainer="Alastair Bradford",
    maintainer_email="team@qutmotorsport.com",
    description="Consolidates and compaires cone locations",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fusion = sensor_fusion.node_fusion:main",
            "track_server = sensor_fusion.node_track_server:main",
        ],
    },
)
