import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'adr_vio'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='OpenVINS 기반 VIO — 설정 생성, map 정렬, rviz 궤적',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vio_align = adr_vio.vio_align:main',
        ],
    },
)
