from setuptools import setup

package_name = 'demo_package'
submodule_name = 'demo_package/sub_module'

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
    description='Simulator control in python',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = demo_package.main:main'
        ],
    },
)
