from setuptools import setup

package_name = 'ml_training'

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
    maintainer='QUT Motorsport',
    maintainer_email='qutmotorsport.team@gmail.com',
    description='ML training for YOLOv5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'training = ml_training.training:main'
        ],
    },
)
