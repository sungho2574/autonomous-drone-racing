import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'adr_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='PX4 offboard 제어. step1: TrajectorySetpoint 기반 position control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_position_controller = adr_control.px4_position_controller:main',
        ],
    },
)
