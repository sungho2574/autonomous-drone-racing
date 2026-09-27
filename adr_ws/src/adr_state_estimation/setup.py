import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'adr_state_estimation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='VIO drift 를 게이트 PnP 로 보정하는 KF (논문 §2.4). 시각화 전용',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drift_corrector = adr_state_estimation.drift_corrector:main',
        ],
    },
)
