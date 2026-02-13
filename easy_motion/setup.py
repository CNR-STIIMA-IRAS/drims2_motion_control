import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'easy_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools',
                      'easy_motion_msgs'],
    zip_safe=True,
    maintainer='kalman',
    maintainer_email='samuele.sandrini@stiima.cmr.it',
    description='ROS 2 Motion server and client utilities for MoveIt2-based manipulation and end-effector support, including some utils',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'motion_server = {package_name}.motion_server:main',
            f'motion_client_test = examples.motion_client_test:main',
            f'fake_dice_identification_server = examples.fake_dice_identification_server:main',
        ],
    },
)
