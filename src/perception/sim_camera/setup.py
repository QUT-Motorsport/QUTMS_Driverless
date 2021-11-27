from setuptools import setup

package_name = 'sim_camera'

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
    maintainer='Alastair Bradford, Harl Towne',
    maintainer_email='qutmotorsport.team@gmail.com',
    description='Simulator camera processing in Python',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_cam_proc = sim_camera.node_cam_proc:main'
        ],
    },
)
