from setuptools import setup

package_name = "sbg_testing"

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
    maintainer="Zac Gillerat",
    maintainer_email="team@qutmotorsport.com",
    description="Testing functions of the SBG Ellipse",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sbg_performance = sbg_testing.sbg_performance:main",
        ],
    },
)
