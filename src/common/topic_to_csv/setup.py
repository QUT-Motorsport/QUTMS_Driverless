from setuptools import setup

package_name = "topic_to_csv"
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
    description="Writes topics to CSV",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "topic_to_csv = topic_to_csv.node_topic_to_csv:main",
        ],
    },
)
