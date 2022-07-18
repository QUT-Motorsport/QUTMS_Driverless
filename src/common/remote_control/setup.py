from setuptools import setup

package_name = "remote_control"

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
    maintainer="Kai Thouard",
    maintainer_email="team@qutmotorsport.com",
    description="keyboard control for simulation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "key_control = remote_control.node_key_control:main",
            "key_control_reset = remote_control.node_key_control_reset:main",
            "key_pygame = remote_control.node_key_pygame:main",
        ],
    },
)
