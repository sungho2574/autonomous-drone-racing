import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'adr_perception'

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
    description='게이트 인식 (색상 기반 2D 검출 → GateDetectionArray; step3: Gatenet + PnP)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gate_detector = adr_perception.gate_detector:main',
        ],
    },
)
