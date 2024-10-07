from glob import glob
import os

from setuptools import setup

package_name = "vehicle_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        ("share/" + package_name + "/" + os.path.join(os.path.dirname(__file__)), ["package.xml"]),
        # (os.path.join("share", package_name), glob("/../../hardware/QUTMS_Embedded_Common/QUTMS.dbc")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alastair Bradford, Walden Leung, Alex Bradbury",
    maintainer_email="team@qutmotorsport.com",
    description="Vehicle operations and supervisor",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vehicle_supervisor_node = vehicle_bringup.node_vehicle_supervisor:main",
            "inspection_handler_node = vehicle_bringup.node_inspection_handler:main",
            "trackdrive_handler_node = vehicle_bringup.node_trackdrive_handler:main",
            "ebs_test_handler_node = vehicle_bringup.node_ebs_test_handler:main",
        ],
    },
)
