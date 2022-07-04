from setuptools import setup

package_name = 'lidar_pipeline'
submodule_name = 'lidar_pipeline/scripts'

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
    maintainer='Liam Ferrante, Alastair Bradford',
    maintainer_email='qutmotorsport.team@gmail.com',
    description='LiDAR analysis in python',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_perception = lidar_pipeline.node_processing:main',
            'sim_simple = lidar_pipeline.node_simple:main'
        ],
    },
)
