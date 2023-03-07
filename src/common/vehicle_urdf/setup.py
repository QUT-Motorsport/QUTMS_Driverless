from glob import glob
import os

from setuptools import find_packages, setup

package_name = "vehicle_urdf"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.dae")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf.xacro")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alastair Bradford",
    maintainer_email="team@qutmotorsport.com",
    description="QUTMS Driverless vehicle URDF files",
    license="MIT",
    tests_require=["pytest"],
)
