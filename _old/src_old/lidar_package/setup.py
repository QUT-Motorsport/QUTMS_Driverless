from setuptools import setup

package_name = 'lidar_package'
submodule_name = 'lidar_package/sub_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='QUT Motorsport',
    maintainer_email='qutmotorsport.team@gmail.com',
    description='LiDAR analysis in python',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_math = lidar_package.lidar_math:main'
        ],
    },
)
