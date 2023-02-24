from setuptools import setup

package_name = "steering_testing"

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
    maintainer="Alastair Bradford, Stephen Wardle, Ryan Best",
    maintainer_email="team@qutmotorsport.com",
    description="Steering actuator testing project",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "random = steering_testing.node_random:main",
            "step_response = steering_testing.node_step_response:main",
            "step_response_calibration = steering_testing.node_step_response_calibration:main",
        ],
    },
)
