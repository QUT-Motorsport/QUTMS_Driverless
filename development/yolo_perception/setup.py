from setuptools import setup

package_name = 'yoloController'

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
    maintainer='Lachlan Masson',
    maintainer_email='lachlanmasson@gmailcom',
    description='A package to detect cones from a topic using yolov5',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'firstNode = yoloController.firstNode:main',
            'detectCones = yoloController.detectCones:main',
            'testConesTopic = yoloController.yoloTester:main'
        ],
    },
)
