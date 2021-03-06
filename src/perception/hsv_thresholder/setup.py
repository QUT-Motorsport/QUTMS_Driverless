from setuptools import setup

package_name = "hsv_thresholder"

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
    description="HSV Thresholding package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "thresholder = hsv_thresholder.node_thresholder:main",
            "gui = hsv_thresholder.node_gui:main",
        ],
    },
)
