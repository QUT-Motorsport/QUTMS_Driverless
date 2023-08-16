from setuptools import setup

package_name = "terminal_control"

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
    maintainer="Alistair English, Alastair Bradford",
    maintainer_email="team@qutmotorsport.com",
    description="Keyboard based controller that publishes drive and state messages",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller = terminal_control.node_controller:main",
            "state_controller = terminal_control.node_state_controller:main"
        ],
    },
)
