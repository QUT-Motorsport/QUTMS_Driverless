from setuptools import setup

package_name = "odom_transformer"

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
    description="SBG UTM to odom frame transform",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_transformer_node = odom_transformer.node_odom_transformer:main",
            "odom_rotater_node = odom_transformer.node_odom_rotater:main",
        ],
    },
)
