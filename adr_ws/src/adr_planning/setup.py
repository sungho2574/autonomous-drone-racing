import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'adr_planning'

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
    description='min-snap 궤적 생성 (step1). 웨이포인트 = 게이트 맵(gates.yaml)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gate_planner = adr_planning.gate_planner_node:main',
            'plot_trajectory = adr_planning.plot_trajectory:main',
        ],
    },
)
