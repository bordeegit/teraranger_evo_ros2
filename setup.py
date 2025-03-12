from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'teraranger_evo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial', 'crcmod'],
    zip_safe=True,
    maintainer='Matteo Bordignon',
    maintainer_email='matteo.bordignon@polimi.it',
    description='ROS2 driver for TeraRanger Evo sensor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'teraranger_evo_node = teraranger_evo.teraranger_evo_node:main',
        ],
    },
)
