from setuptools import setup

package_name = 'sim_translator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alistair English, Alastair Bradford',
    maintainer_email='team@qutmotorsport.com',
    description='Translates FS Sim topics to Driverless topics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cone_detection_translator = sim_translator.node_cone_detection_translator:main",
        ],
    },
)
