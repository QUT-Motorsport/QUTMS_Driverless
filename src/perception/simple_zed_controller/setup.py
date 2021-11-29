from setuptools import setup

package_name = 'simple_zed_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alistair English, Alastair Bradford',
    maintainer_email='qutmotorsport.team@gmail.com',
    description='ZED2i depth processing + control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = simple_zed_controller.node_controller:main',
            'thresholder = simple_zed_controller.node_thresholder:main',
        ],
    },
)
