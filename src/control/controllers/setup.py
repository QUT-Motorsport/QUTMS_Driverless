from setuptools import setup

package_name = "controllers"

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
            "sine = controllers.node_sine:main",
            "reactive_control_node = controllers.node_reactive_control:main",
            "reactive_control_lifecycle = controllers.lifecycle_reactive_control:main",
            "point_fitting = controllers.node_point_fitting:main",
            "vector_spline = controllers.node_vector_spline:main",
            "point_fitting2 = controllers.node_point_fitting2:main",
            "vel_to_ackermann = controllers.node_vel_to_ackermann:main",
        ],
    },
)
