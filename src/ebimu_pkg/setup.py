from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ebimu_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e2box',
    maintainer_email='e2b@e2box.co.kr',
    description='EBIMU ROS2 publisher, EKF launch files, and elevator floor estimator',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ebimu_publisher = ebimu_pkg.ebimu_publisher:main',
            'ebimu_subscriber = ebimu_pkg.ebimu_subscriber:main',
            'elevator_floor_node = ebimu_pkg.elevator_floor_node:main',
        ],
    },
)