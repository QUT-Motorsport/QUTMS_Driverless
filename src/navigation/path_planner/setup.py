from setuptools import setup

package_name = 'path_planner'

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
    maintainer='Alastair Bradford',
    maintainer_email='team@gqutmotorsport.com',
    description='Path Planner for mapped tracks',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_planner = path_planner.node_planner:main'
        ],
    },
)
