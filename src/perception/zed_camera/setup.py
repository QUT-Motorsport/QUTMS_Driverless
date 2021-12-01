from setuptools import setup

package_name = "zed_camera"

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
    description="ZED2i package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller = zed_camera.node_controller:main",
            "thresholder = zed_camera.node_thresholder:main",
            "detector = zed_camera.node_detector:main",
        ],
    },
)
