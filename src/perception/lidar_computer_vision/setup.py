from setuptools import setup

package_name = 'lidar_computer_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Liam Ferrante',
    maintainer_email='qutmotorsport.team@gmail.com',
    description='Real-time LiDAR computer vision pipeline for autonomous formula student racecar.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_cv = lidar_computer_vision.lidar_cv:main'
        ],
    },
)
