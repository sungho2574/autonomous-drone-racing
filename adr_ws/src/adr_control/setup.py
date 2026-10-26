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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'policies', 'a2rl_monorace'),
         glob('policies/a2rl_monorace/*')),
        *[(os.path.join('share', package_name, 'policies', name), glob('policies/' + name + '/*'))
          for name in ('A_slow', 'B_fast')],
    ],
    install_requires=['setuptools'],
    extras_require={'rl': ['onnxruntime==1.23.2', 'numpy<2', 'PyYAML']},
    zip_safe=True,
    description='PX4 offboard 제어. step1: TrajectorySetpoint 기반 position control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a2rl_rl_controller = adr_control.a2rl_rl_controller:main',
            'px4_position_controller = adr_control.px4_position_controller:main',
        ],
    },
)
